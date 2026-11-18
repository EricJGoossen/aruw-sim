#!/usr/bin/env python3
"""Runs the hub in-process and prints what the firmware believes next to sim
truth, for settling sign conventions and checking a run.

    scripts/compare.py [--backend achilles|mujoco] [--runtime-dir DIR]
                       [--seconds N] [--period S] [--lockstep --expect mcb,otto
                       --ack-timeout S]

scripts/run.sh --compare starts it in place of the hwemu binary (same hub, same
ports), then starts the MCB and otto as usual. Every --period wall seconds it
prints:
- the firmware's belief: the chassis and turret odometry the MCB last sent
  otto (from the vision link's log, <run dir>/vision.jsonl);
- sim truth: the chassis's and the turret minor's attitude, the joint
  encoders and the world yaw rates.
At the end it writes <run dir>/compare.json, a summary scripts/smoke.sh checks:
how far the chassis turned in truth, how far the MCB thinks it travelled,
whether odometry arrived, and the last of both.

Needs the compiled hwemu module (python/hwemu/_hwemu*.so, built by build.sh),
so it runs with the interpreter the module was built for (the container's
python3).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "hw-emulator", "python"))
import hwemu  # noqa: E402

ASSETS = os.path.join(ROOT, "assets", "sentry")
ENCODERS = ["major_yaw_motor_encoder", "minor_yaw_encoder", "minor_pitch_encoder", "flywheel_left_encoder",
            "agitator_encoder", "lf_wheel_encoder"]
POSE = ["x", "y", "z", "roll", "pitch", "yaw"]


def last_odometry(path: str, offset: int) -> tuple[dict | None, int, int]:
    """The newest odometry message in the vision log after byte `offset`, the
    new offset, and how many odometry messages were seen."""
    odo, count = None, 0
    try:
        with open(path) as f:
            f.seek(offset)
            for line in f:
                if '"odometry"' in line:
                    try:
                        odo = json.loads(line)["msg"]
                        count += 1
                    except (json.JSONDecodeError, KeyError):
                        pass
            offset = f.tell()
    except FileNotFoundError:
        pass
    return odo, offset, count


def unwrap(prev: float | None, now: float, acc: float) -> tuple[float, float]:
    if prev is None:
        return now, acc
    d = (now - prev + math.pi) % (2 * math.pi) - math.pi
    return now, acc + abs(d)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--backend", default="achilles", choices=["achilles", "mujoco"])
    ap.add_argument("--runtime-dir", default=os.path.join(ROOT, "build", "run", "compare"))
    ap.add_argument("--seconds", type=float, default=None, help="sim seconds (default: until Ctrl-C)")
    ap.add_argument("--period", type=float, default=2.5, help="wall seconds between printouts")
    ap.add_argument("--lockstep", action="store_true")
    ap.add_argument("--expect", default=None, help="lockstep participants to wait for, comma-separated")
    ap.add_argument("--ack-timeout", type=float, default=None)
    args = ap.parse_args()

    def on_term(*_):  # run.sh stops it with SIGTERM: still write the summary
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    rt = args.runtime_dir
    os.makedirs(rt, exist_ok=True)
    hub = hwemu.Hub(os.path.join(ASSETS, "sim.yaml"), os.path.join(ASSETS, "devices.yaml"), args.backend, rt,
                    lockstep=args.lockstep, expect=args.expect.split(",") if args.expect else None,
                    ack_timeout=args.ack_timeout, seconds=args.seconds)
    hub.start()
    s = hub.sim
    print(f"[compare] hub ({args.backend}) running in-process, runtime dir {rt}", flush=True)

    vision_log = os.path.join(rt, "vision.jsonl")
    offset, odo_count = 0, 0
    odo = None
    yaw_prev, yaw_travel = None, 0.0
    first_xy, xy_travel, prev_xy = None, 0.0, None
    try:
        while hub.running():
            time.sleep(args.period)
            new, offset, n = last_odometry(vision_log, offset)
            odo_count += n
            odo = new or odo
            catt = s.attitude("chassis_attitude").read()
            watt = s.attitude("widow_attitude").read()
            yaw_prev, yaw_travel = unwrap(yaw_prev, catt.yaw, yaw_travel)
            enc = "  ".join(f"{e.split('_encoder')[0]}={s.encoder(e).read().position:+.2f}" for e in ENCODERS)
            if odo:
                ch = dict(zip(POSE, odo.get("chassis", [0] * 6)))
                tu = dict(zip(POSE, (odo.get("turrets") or [[0] * 6])[0]))
                xy = (ch.get("x", 0.0), ch.get("y", 0.0))
                if first_xy is None:
                    first_xy = xy
                if prev_xy is not None:
                    xy_travel += math.hypot(xy[0] - prev_xy[0], xy[1] - prev_xy[1])
                prev_xy = xy
                print(f"   MCB says:  chassis yaw={ch.get('yaw', 0):+.2f} xy=({xy[0]:+.2f},{xy[1]:+.2f})"
                      f"  turret yaw={tu.get('yaw', 0):+.2f} pitch={tu.get('pitch', 0):+.2f}")
            else:
                print("   MCB says:  (no odometry on the vision link yet)")
            print(f"   sim truth: chassis yaw={catt.yaw:+.2f}                    "
                  f"turret yaw={watt.yaw:+.2f} pitch={watt.pitch:+.2f}")
            print(f"t={s.time():6.2f} {enc}  wz chassis={s.gyro('chassis_gyro').read().z:+.2f} "
                  f"major={s.gyro('major_ism330a_gyro').read().z:+.2f} widow={s.gyro('widow_gyro').read().z:+.2f}",
                  flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        sim_time = s.time()
        hub.stop()
        summary = {
            "backend": args.backend,
            "sim_time": sim_time,
            "odometry_messages": odo_count,
            "truth_chassis_yaw_travel": yaw_travel,
            "mcb_chassis_xy_travel": xy_travel,  # summed between printouts: noisy while spinning
            "mcb_chassis_xy_net": math.hypot(prev_xy[0] - first_xy[0], prev_xy[1] - first_xy[1]) if first_xy else 0.0,
            "mcb_chassis_xy_first": first_xy,
            "mcb_chassis_xy_last": prev_xy,
            "mcb_last_odometry": odo,
        }
        with open(os.path.join(rt, "compare.json"), "w") as f:
            json.dump(summary, f, indent=1)
        print(hub.status(), flush=True)
        print(f"[compare] {json.dumps({k: v for k, v in summary.items() if k != 'mcb_last_odometry'})}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
