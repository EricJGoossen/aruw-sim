"""Puts a Python process (otto) on the hub's sim time. Loaded automatically:
scripts/run.sh puts aruw-sim's shims/ on otto's PYTHONPATH, and Python imports
`sitecustomize` at startup.

Only with HWEMU_RUNTIME_DIR set (a sim run), and HWEMU_SIM_TIME not "0":

- **Sim time.** time.time / time_ns, monotonic / monotonic_ns, perf_counter /
  perf_counter_ns, clock_gettime / clock_gettime_ns (the realtime and
  monotonic clocks) and sleep read and wait on the hub's clock
  (hwemu.clock.SimClock), so a process stopped at a breakpoint, or a sim
  running slower than real time, never sees a timeout it wouldn't see on the
  robot. time.time keeps a wall-clock epoch: it is the wall time at startup
  plus the sim time since. It waits up to HWEMU_CLOCK_WAIT seconds (default
  30) for the hub's clock at startup; without one, time stays real (a gap).
  The debugger's own threads (debugpy / pydevd) keep real time, or they'd
  freeze with the sim at a breakpoint.
- **Lockstep.** With HWEMU_LOCKSTEP set, a thread joins the hub's lockstep as
  $HWEMU_PROCESS (default "otto") and acks every tick
  (hwemu.clock.LockstepParticipant): the hub waits for it, so pausing this
  process in a debugger freezes the whole sim.
- **No armor detector.** otto's IconTargetDetector is CUDA/TensorRT-only;
  when otto imports it, shims/hwemu_detector_stub.py's stand-in replaces it:
  no torch, no CUDA, no detections, and the gap `detector: disabled`, which
  scripts/smoke.sh fails on.
- **True targets** (HWEMU_TRUTH_TARGETS set; run.sh --enemy sets it): otto's
  turret selection is handed the enemy's true state instead
  (shims/hwemu_truth_targets.py), so its auto-nav reacts to the enemy.
- **Jetson-only wheels** the 2026 comp branch imports have stand-ins here:
  rm_pose_solver and sumedh_quick_uint16_median (otto main's own Python
  ports) and robomaster_particle_filters (import-only).
- **Gaps.** Log records carrying "[hwemu GAP] source: message" (what the otto
  patch logs) are recorded in <runtime dir>/gaps/$HWEMU_PROCESS.jsonl, like
  hwemu.gaps.report's.

What stays on wall time, reported once as a gap: timeouts inside threading,
queue, select, socket and serial waits (C code that reads the clock itself).
"""

import os
import sys


def _install() -> None:
    import logging
    import threading
    import time

    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    hwemu_py = os.path.join(repo, "hw-emulator", "python")
    if hwemu_py not in sys.path:
        sys.path.append(hwemu_py)

    from hwemu import clock as hclock
    from hwemu import gaps

    os.environ.setdefault("HWEMU_PROCESS", "otto")

    # [hwemu GAP] log lines -> gap reports, whatever handlers the app installs.
    original_handle = logging.Logger.handle
    gap_handler = gaps.GapLogHandler()

    def handle(self, record):  # type: ignore[no-untyped-def]
        if isinstance(record.msg, str) and "[hwemu GAP]" in record.msg:
            gap_handler.emit(record)
        return original_handle(self, record)

    logging.Logger.handle = handle  # type: ignore[method-assign]

    _install_otto_hooks(here, repo)

    if os.environ.get("HWEMU_SIM_TIME", "1") == "0":
        return

    sim = hclock.process_clock().sim
    wait = float(os.environ.get("HWEMU_CLOCK_WAIT", "30"))
    deadline = hclock._real_monotonic() + wait
    while not sim.available() and hclock._real_monotonic() < deadline:
        hclock._real_sleep(0.05)
    if not sim.available():
        gaps.report("sitecustomize", "no-clock", f"no hub clock after {wait:g} s: this process runs on wall time")
        return

    real = {name: getattr(time, name) for name in
            ("time", "time_ns", "monotonic", "monotonic_ns", "perf_counter", "perf_counter_ns", "sleep",
             "clock_gettime", "clock_gettime_ns")}
    pc = hclock.process_clock()
    epoch = pc.epoch()

    def _debugger_thread() -> bool:
        if getattr(threading.current_thread(), "is_pydev_daemon_thread", False):
            return True
        f = sys._getframe(2)
        fn = f.f_code.co_filename if f is not None else ""
        return "pydevd" in fn or "debugpy" in fn

    def _now_us() -> int:
        us = pc.micros()
        return us if us is not None else 0

    def _now() -> float:
        return _now_us() * 1e-6

    def sim_time() -> float:
        return real["time"]() if _debugger_thread() else epoch + _now()

    def sim_time_ns() -> int:
        return real["time_ns"]() if _debugger_thread() else int(epoch * 1e9) + _now_us() * 1000

    def sim_monotonic() -> float:
        return real["monotonic"]() if _debugger_thread() else _now()

    def sim_monotonic_ns() -> int:
        return real["monotonic_ns"]() if _debugger_thread() else _now_us() * 1000

    def sim_perf_counter() -> float:
        return real["perf_counter"]() if _debugger_thread() else _now()

    def sim_perf_counter_ns() -> int:
        return real["perf_counter_ns"]() if _debugger_thread() else _now_us() * 1000

    def sim_sleep(seconds: float) -> None:
        if seconds < 0:
            raise ValueError("sleep length must be non-negative")
        if _debugger_thread() or seconds == 0:
            real["sleep"](seconds)
            return
        target = _now() + seconds
        while True:
            left = target - _now()
            if left <= 0:
                return
            # Wake often enough to stay within a tick of the target, without
            # spinning: the sim can't advance more than this in wall time
            # unless it runs faster than real time.
            real["sleep"](min(max(left, 0.0002), 0.002))

    # clock_gettime(_ns) on the wall and monotonic clocks, as the functions
    # above (the 2026 comp branch's JetsonTimestamp reads CLOCK_REALTIME this
    # way); other clocks (CPU time) stay real.
    sim_clocks = {
        time.CLOCK_REALTIME: (sim_time, sim_time_ns),
        time.CLOCK_MONOTONIC: (sim_monotonic, sim_monotonic_ns),
    }
    for name in ("CLOCK_MONOTONIC_RAW", "CLOCK_BOOTTIME"):
        if hasattr(time, name):
            sim_clocks[getattr(time, name)] = (sim_monotonic, sim_monotonic_ns)

    def sim_clock_gettime(clk_id: int) -> float:
        f = sim_clocks.get(clk_id)
        return f[0]() if f is not None else real["clock_gettime"](clk_id)

    def sim_clock_gettime_ns(clk_id: int) -> int:
        f = sim_clocks.get(clk_id)
        return f[1]() if f is not None else real["clock_gettime_ns"](clk_id)

    time.clock_gettime = sim_clock_gettime
    time.clock_gettime_ns = sim_clock_gettime_ns
    time.time = sim_time
    time.time_ns = sim_time_ns
    time.monotonic = sim_monotonic
    time.monotonic_ns = sim_monotonic_ns
    time.perf_counter = sim_perf_counter
    time.perf_counter_ns = sim_perf_counter_ns
    time.sleep = sim_sleep
    gaps.report("sitecustomize", "interpolated",
                "sim time between hub ticks (1 ms) is interpolated on the wall clock and kept strictly "
                "increasing, so timestamps within a tick aren't the hub's exact time")
    gaps.report("sitecustomize", "wall-timeouts",
                "time.* runs on sim time, but timeouts inside threading, queue, select, socket and serial waits "
                "still run on wall time")

    if os.environ.get("HWEMU_LOCKSTEP"):
        name = os.environ.get("HWEMU_PROCESS", "otto")
        hclock.LockstepParticipant(name).start(wait_s=wait)
        print(f"[hwemu] {name}: in lockstep with the hub", file=sys.stderr, flush=True)


def _after_import(names, then) -> None:  # type: ignore[no-untyped-def]
    """Runs then(module) once the first of `names` (one module, under each otto layout:
    the 2026 comp branch's and main's) has loaded."""
    import importlib.abc
    import importlib.util

    class Finder(importlib.abc.MetaPathFinder):
        def find_spec(self, name, path, target_module=None):  # type: ignore[no-untyped-def]
            if name not in names:
                return None
            sys.meta_path.remove(self)  # once is enough; and no recursion below
            spec = importlib.util.find_spec(name)
            if spec is None or spec.loader is None:
                return spec
            exec_module = spec.loader.exec_module

            def exec_then(module):  # type: ignore[no-untyped-def]
                exec_module(module)
                then(module)

            spec.loader.exec_module = exec_then  # type: ignore[method-assign]
            return spec

    sys.meta_path.insert(0, Finder())


def _install_otto_hooks(shims_dir: str, repo: str) -> None:
    """The sim's stand-ins inside otto, as its modules load (either layout)."""
    if shims_dir not in sys.path:
        sys.path.insert(0, shims_dir)

    def stub_detector(module):  # type: ignore[no-untyped-def]
        from hwemu_detector_stub import make_stub

        module.IconTargetDetector = make_stub(module.TargetDetector)

    _after_import(
        ("project_otto.vision.target_detector._target_detector", "project_otto.target_detector._target_detector"),
        stub_detector,
    )

    if os.environ.get("HWEMU_TRUTH_TARGETS", "0") not in ("", "0"):

        def truth_targets(module):  # type: ignore[no-untyped-def]
            import hwemu_truth_targets

            hwemu_truth_targets.install(module, repo)

        _after_import(("project_otto.vision.turret._turret", "project_otto.turret._turret"), truth_targets)


if os.environ.get("HWEMU_RUNTIME_DIR"):
    try:
        _install()
    except Exception as e:  # noqa: BLE001 -- never stop the interpreter from starting
        print(f"[hwemu GAP] sitecustomize: couldn't put this process on sim time: {e!r}", file=sys.stderr)
