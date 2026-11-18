# aruw-sim

The ARUW sentry in simulation: the **real MCB firmware** (aruw-mcb's hosted
build) and **real otto** (project-otto) run as separate processes against one
simulated sentry, served by [hw-emulator](https://github.com/EricJGoossen/hw-emulator)
on either physics backend, [Achilles](https://github.com/EricJGoossen/Achilles)
(the default) or MuJoCo.

```
 aruw-mcb.elf ──CAN (Unix sockets), SPI/GPIO/PWM/ADC (pinbus), UARTs (PTYs)──┐
                                                                             ├── hwemu hub ── Achilles | MuJoCo
 otto ──vision UART, LD19 lidars (PTYs), D435 + Arducam (/dev/shm rings)─────┘      (assets/sentry: URDF → .arow)
```

Nothing the sim doesn't model is silent: every stub, unserved message or
approximation prints `[hwemu GAP] …` once and lands in the run's gap report.

## Quick start

```sh
git clone --recursive https://github.com/EricJGoossen/aruw-sim && cd aruw-sim
git -C project-otto submodule update --init --recursive
git -C aruw-mcb submodule update --init segger-rtt taproot-scripts

source scripts/known_mismatches.sh   # let today's 3 known model mismatches through (see below)
scripts/run.sh                       # build, then hub + MCB + otto on Achilles; Ctrl-C stops all
scripts/run.sh --backend mujoco --window
scripts/smoke.sh                     # the end-to-end check, both backends
```

The scripts need Docker *or* the toolchain in [.devcontainer/Dockerfile](.devcontainer/Dockerfile)
(Ubuntu 24.04: cmake, ninja, g++-11, gtest/gmock, EGL/GL/GLFW, gdb, rr, uv).
Without the toolchain they re-run themselves in that container
([scripts/dev.sh](scripts/dev.sh)), which mounts the checkout at its own path
and shares the host's network, IPC and display (X11/WSLg), so windows open
and paths match inside and out.

## Scripts

| | |
|---|---|
| [`build.sh`](scripts/build.sh) `[--profile debug\|release] [--only STEP,…] [-j N]` | applies `patches/` idempotently, doctors the URDF and builds the Achilles model, runs `check_model.py`, builds the hub + Achilles + MuJoCo + `libaruw_devices` (CMake/Ninja/ccache), the firmware (`scons build-sim robot=SENTRY_ACHLYS`) and otto's venv (`uv sync --frozen`). Incremental; `-j 2` by default, capped by memory |
| [`run.sh`](scripts/run.sh) `[--backend achilles\|mujoco] [--window] [--lockstep] [--debug mcb\|otto\|hub] [--rr] [--seconds N] [--compare] [--no-build]` | builds, then starts the hub, the MCB and otto; per-process logs in `build/run/<backend>/logs/`, prefixed output on the console; if one dies the rest stop; prints the gap report at the end |
| [`smoke.sh`](scripts/smoke.sh) `[--seconds 30] [--backend …]` | lockstep, headless runs on each backend, checked: MCB boot, both turret-MCB links and IMU calibration, chassis rotation (truth) and translation (MCB odometry), otto's odometry/lidar/frames/referee, and no gap outside `assets/sentry/known_gaps.yaml` |
| [`gaps.sh`](scripts/gaps.sh) `[RUN_DIR] [--check]` | a run's gaps from every process (hub, MCB, otto, model), new ones marked; writes `gaps.json` |
| [`compare.py`](scripts/compare.py) (`run.sh --compare`) | the hub in-process, printing the firmware's odometry next to sim truth (for settling sign conventions); writes `compare.json` |

## Debugging

Every process runs on sim time. `--debug X` implies `--lockstep` with no ack
timeout, so a breakpoint anywhere freezes the whole sim and nothing times out:

- `run.sh --debug mcb`: the firmware (built `-O0 -g` by the debug profile)
  under gdbserver :2345 → VS Code **Attach MCB**. `--rr` records it instead
  (**Replay MCB (rr)**, for stepping backwards).
- `run.sh --debug otto`: otto under debugpy :5678 → **Attach otto**. debugpy
  stops every thread, including otto's lockstep thread, so the hub waits.
  otto's `time.*` is sim time ([shims/sitecustomize.py](shims/sitecustomize.py)),
  so nothing jumps forward on resume.
- `run.sh --debug hub`: the hub under gdbserver :2347 → **Attach hub**.
- **Attach MCB + otto**: both at once.

gdb comes from the container when the host has none ([scripts/gdb.sh](scripts/gdb.sh)).

## Layout

```
achilles/ hw-emulator/ aruw-mcb/ project-otto/   submodules (mcb and otto at upstream GitLab, unpatched)
patches/        aruw-mcb.patch, project-otto.patch: only the robot code's side of the emulator
                interface, all under HWEMU; build.sh applies them
aruw_devices/   libaruw_devices.so, the hub plugin: RMUL referee, cap bank, voltage/current sensor
assets/sentry/  sentry_description/ (the CAD export), urdf_doctor.yaml, devices.yaml, sim.yaml,
                otto_sim.yaml, known_gaps.yaml
shims/          gi/ (GStreamer for otto's Arducam), sitecustomize.py (sim time, lockstep)
tools/          doctor_urdf.py, check_model.py, gap_report.py
scripts/        build.sh run.sh smoke.sh gaps.sh compare.py dev.sh gdb.sh known_mismatches.sh
.vscode/        launch.json, tasks.json
```

## The model

`tools/doctor_urdf.py` turns the vendored URDF plus `assets/sentry/urdf_doctor.yaml`
into the URDF the sim loads (`build/assets/sentry/sentry_{mujoco,achilles}.urdf`):
sensor frames, omni-wheel rollers, flywheels and agitator, armor plates, a
floor and a floating base. For Achilles, Achilles's `tools/urdf_to_arow.py`
converts it to `.arow`, and hw-emulator's `tools/elements_to_arow.py` turns
`devices.yaml`'s `bind.mujoco` elements into Achilles attachments, so one
`devices.yaml` serves both backends.

`tools/check_model.py` is the only code that reads the MCB headers and otto's
config; it compares them with the model and **stops the build** on any
mismatch. Today it reports three, left for the owner to resolve (see
[docs/PLAN.md](docs/PLAN.md)):

- wheelbase: URDF 0.2216 m vs MCB `WHEELBASE_RADIUS` 0.23864 m;
- minor yaw travel: URDF ±90° vs firmware ±143.5°;
- minor pitch travel: URDF −20…35° vs firmware −13…40°.

`source scripts/known_mismatches.sh` lets exactly those through
(`HWEMU_ACCEPT_MISMATCH`); they're still printed.

## Status

See [docs/PLAN.md](docs/PLAN.md) for what's done, what's open, and the
decisions waiting on the owner.
