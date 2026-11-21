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

otto's armor detector is CUDA/TensorRT-only, so the sim stubs it out
([shims/hwemu_detector_stub.py](shims/hwemu_detector_stub.py)): no detections,
and `smoke.sh` fails on it until real detection runs here.

Nothing the sim doesn't model is silent: every stub, unserved message or
approximation prints `[hwemu GAP] …` once and lands in the run's gap report.

## Quick start

```sh
git clone https://github.com/EricJGoossen/aruw-sim && cd aruw-sim
git submodule update --init --recursive
scripts/setup.sh                     # or --otto all: see "Otto's targets" below
scripts/run.sh                       # hub + MCB + otto on Achilles; Ctrl-C stops all
scripts/run.sh --backend mujoco --window --enemy patrol
```

The host needs git, Docker, and Python 3 with PyYAML (`apt install
python3-yaml`), plus access to ARUW's GitLab for aruw-mcb and project-otto.
`setup.sh` checks those, then builds everything in the dev container
([.devcontainer/Dockerfile](.devcontainer/Dockerfile): Ubuntu 24.04 with
cmake, ninja, g++-11, gtest/gmock, EGL/GL/GLFW, gdb, rr, uv). The first run
takes a while; re-running it only redoes what changed. The scripts re-run
themselves in that container ([scripts/dev.sh](scripts/dev.sh)) when the host
lacks the toolchain; it mounts the checkout at its own path and shares the
host's network, IPC and display (X11/WSLg), so windows open and paths match
inside and out.

### Otto's targets

The sim runs one of three otto branches, each with its patch in
`patches/project-otto/` (the firmware has one, `patches/aruw-mcb.patch`):

| `--otto` | branch | where |
|---|---|---|
| `comp` (default) | `feat/comp-branch-sim-integration` | the `project-otto` submodule, at its pinned commit |
| `main` | `main` | a worktree in `build/otto-main` |
| `autonav` | `feature/autonav_framework` (the 2026-27 ROS 2 stack) | a worktree in `build/otto-autonav`, plus its ROS image and packages |

`scripts/setup.sh --otto comp,main,autonav` (or `all`) sets them up;
`scripts/otto_setup.sh TARGET [REF]` sets up one, or moves it to another
commit. Then `scripts/run.sh --otto main`, or `--autonav` (see below).
A patch that no longer applies to its branch stops setup with the file that
failed: re-export it from a working tree (`git diff` there, plus new files).

## Scripts

| | |
|---|---|
| [`build.sh`](scripts/build.sh) `[--profile debug\|release] [--only STEP,…] [-j N]` | applies `patches/` idempotently, doctors the URDF and builds the Achilles model, runs `check_model.py`, builds the hub + Achilles + MuJoCo + `libaruw_devices` (CMake/Ninja/ccache), the firmware (`scons build-sim robot=SENTRY_ACHLYS`) and otto's venv (`uv sync --frozen`). Incremental; `-j 2` by default, capped by memory |
| [`setup.sh`](scripts/setup.sh) `[--otto TARGET,…\|all] [-j N]` | everything a fresh clone needs: checks the host, checks out the submodules, runs `build.sh`, and `otto_setup.sh` for each of otto's other targets |
| [`run.sh`](scripts/run.sh) `[--backend achilles\|mujoco] [--window] [--lockstep] [--debug mcb\|otto\|hub] [--rr] [--seconds N] [--compare] [--no-build] [--enemy BEHAVIOR] [--otto comp\|main\|autonav] [--autonav\|--autonav-drive]` | builds, then starts the hub, the MCB and otto; per-process logs in `build/run/<backend>/logs/`, prefixed output on the console; if one dies the rest stop; prints the gap report at the end. `--enemy patrol` adds a blue sentry the hub drives by `assets/enemy/behaviors/patrol.yaml` (default: none). `--autonav` runs the 2026-27 ROS 2 autonav stack instead of the comp branch's (see below) |
| [`otto_setup.sh`](scripts/otto_setup.sh) `comp\|main\|autonav [REF]` | one of otto's targets: a worktree of project-otto at the branch (or REF) in `build/otto-TARGET` with its patch, its venv, and for autonav the `aruw-autonav:humble` image (`docker/autonav`) and its ROS packages in `build/autonav-ws`. comp is the submodule, which `build.sh` handles |
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
patches/        aruw-mcb.patch, project-otto/{comp,main,autonav}.patch: only the robot code's side of the emulator
                interface, all under HWEMU; build.sh applies them
aruw_devices/   libaruw_devices.so, the hub plugin: RMUL referee, cap bank, voltage/current sensor
assets/sentry/  sentry_description/ (the CAD export), urdf_doctor.yaml, devices.yaml, sim.yaml,
                otto_sim.yaml, known_gaps.yaml
assets/enemy/   enemy.yaml (the opponent sentry run.sh --enemy adds), behaviors/ (its routes)
shims/          gi/ (GStreamer for otto's Arducam), sitecustomize.py (sim time, lockstep)
tools/          doctor_urdf.py, check_model.py, gap_report.py, enemy_model.py,
                field_map.py (the field as a ROS occupancy map), autonav/ (run.sh --autonav's ROS side)
docker/autonav/ the ROS 2 Humble image run.sh --autonav runs otto and the ROS nodes in
scripts/        setup.sh otto_setup.sh build.sh run.sh smoke.sh gaps.sh compare.py dev.sh gdb.sh known_mismatches.sh
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
mismatch. Today it reports three, left for the owner to resolve:

- wheelbase: URDF 0.2216 m vs MCB `WHEELBASE_RADIUS` 0.23864 m;
- minor yaw travel: URDF ±90° vs firmware ±143.5°;
- minor pitch travel: URDF −20…35° vs firmware −13…40°.

`build.sh` lets exactly those through by default
(`scripts/known_mismatches.sh`); they're still printed, and
`HWEMU_ACCEPT_MISMATCH=` (empty) makes the check strict.

## The 2026-27 autonav stack (`run.sh --autonav`)

project-otto's `feature/autonav_framework` moves navigation to ROS 2: AMCL
localizes the sentry from its Odin lidar against a field map, and a bridge
inside otto sends each fix to the MCB as the ArUco-localization message, which
the MCB's wheel EKF fuses. `scripts/setup.sh --otto autonav` once, then
`scripts/run.sh --autonav` runs it against the sim:

- The hub and the MCB run on the host (not the dev container), so that otto
  and the ROS nodes, each in an `aruw-autonav:humble` container on the host's
  network, IPC and PTYs, can reach them.
- The Odin is as `assets/sentry/odin.yaml` has it, from its characterization
  against motion capture (Oct 2026). Its cloud is fans of beams across its
  126-degree field of view at its CAD mount on the turret major (the model's
  `odin` frame), tilted to sample the 0.10-0.40 m band the maps are drawn
  for. Its pose is truth plus its measured error: a registration offset per
  relocalization, correlated heavy-tailed noise, no pose until its first fix
  ~10 s after start, and a 14% chance of a wrong first fix
  (`HWEMU_ODIN_SEED`, default 1, seeds it; `HWEMU_ODIN_WRONG_FIX=1` or `0`
  forces or rules out the wrong fix). `tools/autonav/hwemu_odin.py` stands
  in for its driver (cloud, pose, `/clock`) and logs its fixes and output to
  `<run dir>/autonav/odin_events.jsonl` and `odin_pose.jsonl`.
- The map is this field's (`tools/field_map.py`), not the branch's ARCC 2027
  one: a different field from the sim's 2026 one.
- The branch has no ROS 2 planner yet, so the robot gets no paths.
  `--autonav-drive` also runs the legacy planner (a sim-only switch in
  `patches/project-otto/autonav.patch`) to watch the localization while it
  drives. `scripts/autonav_check.py` reports the localization's error against
  truth from `<run dir>/autonav/localization.jsonl`, and the Odin's own.

