# aruw-sim: plan and hand-off notes

This is the plan the cloud session worked from, with its progress notes, copied
here so a local session can pick up where it stopped. "Progress, second
checkpoint" below is the current state; everything under it that isn't marked
done is still to do.

## Setting up a machine to continue

```sh
git clone --recursive https://github.com/EricJGoossen/aruw-sim
cd aruw-sim
git -C project-otto submodule update --init --recursive   # aruw-lightning-models -> label-api, labels
git -C aruw-mcb submodule update --init segger-rtt taproot-scripts
```

What the cloud session needed installed (Ubuntu 24.04), until `scripts/build.sh` exists to check for them:
- `cmake ninja-build ccache g++` (C++20), and `g++-11`: GCC 13 rejects the hosted modm's protothread macros ("jump to case label"), and taproot's SConscript picks gcc-11 automatically when it's installed
- `libgtest-dev libgmock-dev googletest` (the hosted MCB build links them)
- `libegl-dev libgl-dev libgles-dev libglfw3-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev`
- scons 3.1.2 in a venv (`uv venv --python 3.11 mcbenv && uv pip install scons==3.1.2`), run with `SCONS_LIB_DIR=<venv>/lib/python3.11/site-packages/scons`
- `uv` for otto (`UV_PROJECT_ENVIRONMENT=build/otto-venv uv sync --frozen` in project-otto; needs download-r2.pytorch.org, which the cloud environment blocked)
- build with -j1/-j2 on 16 GB: Achilles' tests and the hub link at several GB per job

How the pieces were built and run by hand:

```sh
# hub + Achilles + plugin (umbrella CMake)
cmake -S . -B build/cmake -G Ninja -DHWEMU_WITH_PYTHON=ON && ninja -C build/cmake -j2
python3 tools/doctor_urdf.py assets/sentry/urdf_doctor.yaml build/assets/sentry
python3 tools/check_model.py            # fails on the 3 known mismatches below; for test runs:
HWEMU_ACCEPT_MISMATCH=wheelbase_radius.joint_fl_wheel,... python3 tools/check_model.py

# MCB: apply patches/aruw-mcb.patch to aruw-mcb, then
mkdir -p build/hwemu_client && cp hw-emulator/client/hwemu_client.h build/cmake/hw-emulator/libhwemu_client.a build/hwemu_client/
cd aruw-mcb/aruw-mcb-project && scons build-sim robot=SENTRY_ACHLYS profile=debug hwemu=../../build/hwemu_client -j2
#   -> aruw-mcb/aruw-mcb-project/build/sim-hwemu/scons-debug/TARGET_SENTRY_ACHLYS/aruw-mcb.elf

# run (MuJoCo until the Achilles converter exists)
build/cmake/hw-emulator/hwemu --sim assets/sentry/sim.yaml --devices assets/sentry/devices.yaml \
    --backend mujoco --runtime-dir /tmp/hwemu --status 5 &
HWEMU_RUNTIME_DIR=/tmp/hwemu HWEMU_PROCESS=mcb <the .elf above>
```

---

# Plan: sentry in sim, with aruw-mcb and project-otto running against hw-emulator

## Context

hw-emulator (main @ e698fe6) and the Achilles branch (`claude/hwemu-support` @ 5895184, which you'll review and merge yourself; I won't merge it) emulate the wire protocols. Nothing runs the real robot code against them yet. The goal is to run the **sentry's real MCB firmware** (hosted build) and **real otto** as separate processes against one simulated sentry.

What you decided:
- **New umbrella repo.** It submodules Achilles, hw-emulator, project-otto and aruw-mcb. It also holds:
  - the sentry assets (URDF, meshes, .arow, sim and device configs, the otto sim overlay) in `assets/`;
  - every **ARUW-specific device that doesn't need sim state** (see "Where code goes").
- **MCB and otto changes are git patch files** for you to review (`patches/`). Each patch contains only the robot code's side of the emulator interface. Robot models and configs never go in those repos.
- **One CAN protocol:** a Unix-socket CAN bus, which works in the cloud without root. SocketCAN is removed from the hub-to-MCB path. You can port it later.
- **Cameras:** a GStreamer (`gi.repository.Gst`) shim plus an `arducam` hub device. The existing pyrealsense2 shim covers the sentry's RealSense.
- **Remote control is out of scope.** The sentry is forced into auto-nav by a clearly commented hack in the MCB patch.
- **Projectiles are out of scope.**
- **Stubs only:** the cap bank, agitator fan, buzzer, OLED, RTT and camera controls get stubs that keep the code from crashing, with no models.
- **Referee:** the RMUL game logic, built from the rules. **Arena:** built from the rules you'll send me.
- **The otto detector:** we just run it on rendered images and see what happens.
- **Robot:** sentry. MCB target `TARGET_SENTRY_ACHLYS`; otto configs `base_config.yaml` + `sentry_name_config.yaml` + `assets/sentry/otto_sim.yaml`.

**Sim gaps are never silent.** Every stub, and every protocol message or feature the sim doesn't serve, logs `[hwemu GAP] …` once, at the moment it first happens. All of them are collected in `<runtime>/gaps.json` and printed at exit. `known_gaps.yaml` lists the expected ones, and `smoke.sh` fails on any other.

## Progress so far (resume from here)

- **Done and pushed (hw-emulator b8ba7b7):**
  - UDS CAN (SocketCAN removed);
  - `gaps.hpp` gap report and client `hwemu_report_gap`;
  - pinbus PWM/ADC and chip-select-routed SPI;
  - device plugins (libhwemu now shared);
  - `arducam` device and the hub `ready` file;
  - 44/44 tests pass.
- **Written, not committed (hw-emulator):** a bus-routed SPI transaction now spans transfers until chip select rises (the ISM330 and MPU6500 drivers clock out the address and the data as separate transfers), with a test. Commit it first.
- **MCB patch, written in `/tmp/claude-0/mcb` (branch hwemu-sim, uncommitted):**
  - new `taproot/src/tap/board/hwemu_hal.{hpp,cpp}`;
  - HWEMU branches in can, uart, digital, analog, pwm, leds and power_outs;
  - the real MPU6500 and ISM330 drivers enabled on emulated SPI;
  - the hosted `Board` gets pin and SPI stand-ins;
  - modm's weak clocks moved onto sim time;
  - lockstep in `main.cpp`;
  - `scons build-sim hwemu=<dir>`, built into `build/sim-hwemu`;
  - the sentry `HWEMU SIM HACK` (forced UP,MID auto-nav, safe-disconnect bypass).

  `aruw-mcb.elf` builds and boots without a hub; each missing link is reported as a gap. Next: export it with `git diff` (plus the untracked new files) to `patches/aruw-mcb.patch`.
- **Findings that shape the remaining work:**
  - The sentry sim build is broken upstream: it's not in aruw-mcb's sim-boot CI, and the hosted `Board` lacks `Ism330Nss` and `DigitalOutPinH`. With HWEMU, the patch supplies them; the plain sim build stays broken. I'll tell you rather than fix it outside HWEMU.
  - The MCB build needs: g++-11 (GCC 13 rejects the protothread macros), system gtest/gmock, the `segger-rtt` and `taproot-scripts` submodules, scons 3.1.2 with `SCONS_LIB_DIR` set, and `robot=SENTRY_ACHLYS` (not `TARGET_…`). `build.sh` handles all of these.
  - The sentry device map, from `sentry_control.cpp`:
    - **CAN1:** minor yaw GM6020 0x206 and pitch 0x205 (both current control), flywheels M3508 0x202 (left) and 0x201 (right), agitator M2006 0x204 (1:36), turret-MCB link 1.
    - **CAN2:** chassis M3508 RF 0x201, LF 0x202, LB 0x203, RB 0x204 (gearbox 268:17); major yaw M3508 0x205 (19.2 × 95/30); Lamprey encoder ID7; CAN encoder ID0; turret-MCB link 2; cap bank; ARUW voltage/current sensor.
    - The values in `sentry_chassis_constants.hpp` disagree with what `sentry_control.cpp` actually instantiates. The sim uses what runs, and `check_model.py` reports the mismatch.
  - **The URDF has no flywheels or agitator.** The doctor YAML adds them as estimated links, because the firmware closes speed and position loops on them.
  - **The ARUW voltage/current sensor (CAN2)** is one more ARUW-only stub for `aruw_devices`. It needs sim state (motor currents), so it reads them through `ISim`.

## Progress, second checkpoint (resume from here)

- **hw-emulator** (pushed a5da992; uncommitted on disk):
  - Pushed: the SPI-transaction change, and MuJoCo `exclude` (body-pattern contact exclusions) and `joint` (armature, damping, frictionloss) elements.
  - Uncommitted: DJI motor `direction` and `encoder_offset`; CAN-encoder `direction`; the EGL context released after each use, so the hub can step the sim from another thread. **Commit these first.**
- **MCB patch:**
  - `patches/aruw-mcb.patch` is exported. It applies to the pinned aruw-mcb, and aruw-mcb's own `scons run-tests` passes with it (475/475).
  - Not yet re-exported: the `remote.cpp` trigger-polling hack (taproot only polls triggers on a DR16 frame) and the OLED/RTT stub gaps. Re-export from `/tmp/claude-0/mcb`.
- **Umbrella** (`/home/user/aruw-sim`; local only, until you create the GitHub repo):
  - submodules (hw-emulator moved to a5da992);
  - top-level `CMakeLists.txt`;
  - `aruw_devices/` (the referee with framing and "in match" telemetry, the cap-bank stub, the voltage/current sensor);
  - `assets/sentry/`: your upload, `urdf_doctor.yaml`, `devices.yaml`, `sim.yaml`;
  - `tools/doctor_urdf.py` and `tools/check_model.py`.
- **Working end to end on MuJoCo:**
  - the hub with all 25 devices;
  - the patched firmware drives all 10 motors, the cap bank, the referee and the vision link, with 19 gaps reported;
  - the chassis spins ("beyblade") while the turret major holds its world yaw;
  - the minor yaw centres once its encoder offset is set.
- **Model findings:**
  - Omni rollers needed spheres, a 32 mm row spacing, armature, and contact exclusions against the chassis hull before the robot could strafe.
  - **`check_model.py` reports 3 mismatches for you to resolve:**
    - wheelbase: URDF 0.2216 m vs MCB 0.23864 m;
    - minor yaw travel: URDF ±90° vs firmware ±143.5°;
    - minor pitch travel: URDF −20…35° vs firmware −13…40°.
  - **Sign conventions are open questions.** Mapping the `inverted` flag to `direction` isn't enough where the URDF's axis convention differs from the firmware's. Evidence: the minor pitch rests on its lower stop while the firmware reads it below its own minimum, and the firmware's chassis yaw stays near 0 while the chassis spins. `scripts/compare.py` (to write, from `/tmp/claude-0/scripts/watch.py`) prints firmware belief against sim truth, from the vision-link log, so you can settle these.
- **Otto:**
  - The patch edits are in the umbrella's `project-otto` working tree: the lidar PTY lookup, and the Arducam V4L skip with gaps. Export them to `patches/project-otto.patch`.
  - **`uv sync` is blocked:** the network refuses `download-r2.pytorch.org` (the lockfile's CPU torch). Next, ask to allow `download.pytorch.org` and `download-r2.pytorch.org` on this environment.
- **Remaining:**
  - otto: the `gi` shim, `sitecustomize` (sim time and the lockstep thread), and hw-emulator `python/hwemu/clock.py`;
  - Achilles `tools/urdf_to_arow.py` plus hw-emulator `tools/elements_to_arow.py` (Achilles is the default backend);
  - `scripts/` (`build.sh`, `run.sh`, `smoke.sh`, `gaps.sh`, `compare.py`), `.vscode`, the README;
  - moving `vision_link` into the umbrella;
  - the referee game and the arena, once your RMUL rules arrive.

## Model: your upload (`sentry_coarse_urdf_export.zip`)

- Unpacked unchanged into `assets/sentry/sentry_description/`.
- **MuJoCo** uses `sentry_coarse_standalone.urdf`, with its convex-hull collision meshes. **Achilles** uses `sentry_primitives.urdf`.
- 7 DOF: four omni wheels on 45° axes, `joint_major`, `joint_minor_yaw` and `joint_minor_pitch`. One turret minor, matching the MCB's `turretWidow`.
- **A doctoring script builds the URDF the sim uses.** `tools/doctor_urdf.py` (umbrella) reads the vendored URDF unchanged, plus `assets/sentry/urdf_doctor.yaml`. It writes `build/assets/sentry/sentry.urdf` (and the primitives variant), and that doctored URDF is the only one the sim loads. Achilles's `urdf_to_arow.py` then generates the .arow from it, so both backends share one model. `build.sh` re-runs the script whenever any of its inputs changes. It adds:
  - **Sensor frames, with their numbers stored in the YAML.** The sim never reads the MCB or otto sources, so the model doesn't feed on the code it tests. For now I copy the values from the code, since there are no separate measurements yet; real measurements can replace them later, and only the YAML changes.
    - IMUs, copied from the MCB headers: `CHASSIS_MCB_{BMI088,ISM330}_MOUNTING_TRANSFORM`, `TURRET_MAJOR_IMU_MOUNTING_TRANSFORM`, `TURRET_MCB1_{BMI088,ISM330}_MOUNTING_TRANSFORM` and `turretMajorImuTransforms`.
    - Camera and lidars, copied from otto's `sentry_name_config.yaml`: `turret_geometry`, `arducam_cameras` and `autonav.lidars`.
  - **Omni-wheel rollers as real URDF links:** N rollers per wheel, each a link on a continuous joint around the rim, with their size and count in the YAML. No new sim element is needed, and Achilles gets them the same way through the generated .arow.
  - **Armor plates, so the vision system sees them:**
    - plate links at RMUL sizes, the light bars beside each plate as emissive geoms in team color, and the ID sticker;
    - their placement on the sentry is estimated from the meshes and marked `estimated: true` in the YAML;
    - the same plate definition is used on the opponents.
- **The consistency check fails loudly.** `tools/check_model.py` (umbrella) runs in `build.sh` and `smoke.sh`, and is the only place the MCB headers and otto's config are read.
  - It compares the doctored URDF and the YAML against them: every IMU mount, the camera and lidar transforms, and the wheel radius (MCB `WHEEL_RADIUS`, 0.0762 m times `FUDGE_FACTOR`) and wheelbase (`WHEELBASE_RADIUS`) against the URDF geometry.
  - Any mismatch beyond a small tolerance **stops the build**, naming the value, both numbers and both files.
  - `--accept-mismatch <name>` is the deliberate override, for once real measurements are in the YAML.
  - With projectiles out of scope, there's no muzzle.

## Where code goes

| Code | Repo | Why |
|---|---|---|
| UDS CAN bus, GapReport, `arducam` device, pinbus PWM/ADC messages, a device-plugin hook | **hw-emulator** | generic, and needs sim state or transport |
| `doctor_urdf.py`, `urdf_doctor.yaml` (frames, rollers, armor plates), `check_model.py` | **umbrella** `tools/`, `assets/` | sentry-specific. Only the check reads the ARUW sources; the sim never does |
| `urdf_to_arow.py` (meshes → boxes and capsules, approximations reported as gaps) | **Achilles** (`tools/`) | it's an Achilles model tool |
| RMUL referee: codec, game logic, ARUW interaction and client-display parsing | **umbrella** `aruw_devices/` | ARUW- and RMUL-specific. It reads the sim state it needs (pose, motor currents) through the public `ISim` API |
| Cap-bank, OLED and RTT stubs | **umbrella** `aruw_devices/` | ARUW-specific, no sim state |
| Fan and buzzer PWM stubs | hw-emulator pinbus | generic pins; it accepts and ignores them, and reports a gap once |

hw-emulator gets a small **plugin hook**: the hub `dlopen`s libraries named under `plugins:` in `devices.yaml`, and each library registers its device types with `DeviceRegistry`. The umbrella builds `libaruw_devices.so` against `hwemu`. hw-emulator stays free of ARUW and RMUL logic.

Existing ARUW-specific devices in hw-emulator: `turret_mcb` needs sim state (the IMU), so it stays. `vision_link` is only a byte relay with ARUW framing for its log, so it moves to the umbrella under the same rule. (Say so if you'd rather leave it.)

## The pieces

1. **hw-emulator**
   - UDS CAN bus plus client functions; SocketCAN removed.
   - GapReport.
   - The plugin hook.
   - `arducam` device.
   - Pinbus PWM and ADC messages.
   - Tests for each.
2. **MCB patch** (`patches/aruw-mcb.patch`). Everything is under `PLATFORM_HOSTED && !ENV_UNIT_TESTS && HWEMU`:
   - **CAN:** `can.cpp` goes through `hwemu_can_*`, replacing `DjiMotorSimHandler`.
   - **UART:** ports map to hub PTYs (vision, referee). The Remote port is a gap.
   - **SPI IMUs:** `mpu6500.cpp`, ISM330 and BMI088 go through `hwemu_spi_transfer`.
   - **GPIO:** digital, analog and PWM go over the pinbus.
   - **Clock:** the clock and delays use `hwemu_clock_micros`.
   - **Lockstep:** `main.cpp` joins it when `HWEMU_LOCKSTEP` is set.
   - **SConstruct:** links `hwemu_client`.
   - **Remote hack:** `sentry_control_operator_interface.cpp` is forced into auto-nav inside a large `// HWEMU SIM HACK` block, reported as a gap at boot.
   - Edits go into the copy of taproot that aruw-mcb keeps in-tree, with the matching upstream taproot files noted.
3. **Otto patch** (`patches/project-otto.patch`):
   - lidar port resolution to hub PTYs;
   - skip `v4l2-ctl` and the device-kill step under HWEMU (reported as a gap);
   - `JetsonTimestamp` on the hub clock;
   - a lockstep thread.

   Outside the patch:
   - the `gi` shim (umbrella `shims/`);
   - a `sitecustomize` that puts otto's `time.*` calls on sim time, so breakpoints never cause timeouts;
   - `otto_sim.yaml`.
4. **Umbrella `aruw_devices/`:** the referee (game logic once your rules arrive; framing and telemetry first), the stubs, and `vision_link`.
5. **Assets:**
   - the doctoring script and its YAML;
   - devices and bindings from `sentry_*constants.hpp` and `sentry_control.cpp` (CAN IDs, buses, motor presets);
   - the .arow generated from the doctored URDF by Achilles's `urdf_to_arow.py`, with mesh-to-primitive approximations reported as gaps;
   - the arena from your rules.
6. **Umbrella scripts, `.vscode` and README.**

## Repo layout (new repo, proposed name `EricJGoossen/aruw-sim`)

The GitHub token in this environment is invalid, so I can't create the repo. Please create it empty, and I'll attach it.

```
aruw-sim/
  achilles/ hw-emulator/ aruw-mcb/ project-otto/   submodules (mcb and otto pinned to upstream GitLab)
  patches/      aruw-mcb.patch, project-otto.patch
  aruw_devices/ referee, cap-bank/OLED/RTT stubs, vision_link (CMake, builds libaruw_devices.so)
  assets/sentry/ sentry_description/ (your upload), devices.yaml, sim.yaml, robot.arow, arena/, otto_sim.yaml, known_gaps.yaml
  shims/        gi/ (Gst), sitecustomize.py
  scripts/      build.sh, run.sh, smoke.sh, gaps.sh
  .vscode/      launch.json, tasks.json
```

- **`build.sh [--profile debug|release] [--only …]`** rebuilds only what changed:
  - CMake/Ninja with ccache for hw-emulator (pointed at the Achilles submodule) and `aruw_devices`;
  - the patches applied idempotently, refusing loudly if one no longer applies;
  - `scons build-sim robot=TARGET_SENTRY_ACHLYS profile=…`;
  - `uv sync`, only when the lockfile changes.

  The job count is capped by available memory.
- **`run.sh [--backend achilles|mujoco] [--window] [--lockstep] [--debug mcb|otto|hub] [--seconds] [--no-build]`** starts the hub, then the MCB, then otto, as separate processes.
  - The backend **defaults to Achilles**.
  - It **runs `build.sh` first**, which is incremental and so near-instant when nothing changed. It doesn't start anything if the build fails. `--no-build` skips the build. Logs go per process, and the console shows prefixed output. One Ctrl-C stops all three, and if one dies, the others stop too.

## Debugging: stepping through the MCB or otto

Everything runs on sim time in lockstep. In debug mode, `ack_timeout` is infinite, so a process stopped at a breakpoint freezes the whole sim, and nothing times out.

- **MCB:** `--profile debug` builds at -O0 with debug info. `run.sh --debug mcb` runs the `.elf` under `gdbserver`, with a VS Code "Attach MCB" config. `--rr` records a run for stepping backwards. A fixed seed with `sequential` lockstep order makes runs reproducible.
- **otto:** `run.sh --debug otto` runs otto under `debugpy --wait-for-client`, with an "Attach otto" config. debugpy stops every thread, including the lockstep one, so the sim freezes. Otto's `time.*` runs on sim time, so nothing jumps forward on resume.
- **Both at once:** a compound config. `--debug hub` runs the hub under gdb.

## Verification

- `build.sh` from a fresh clone builds everything; run again, it rebuilds nothing.
- The hw-emulator ctest passes, plus the new tests. aruw-mcb's `scons run-tests` still passes with the patch applied.
- `smoke.sh` (headless, 30 sim s, on Achilles and then on MuJoCo) checks that:
  - the MCB boots;
  - both turret-MCB links connect;
  - the IMUs calibrate;
  - the chassis strafes and rotates under auto-nav;
  - otto gets odometry, lidar scans and frames;
  - the referee sends game status.

  It fails on any gap not in `known_gaps.yaml`.
- `run.sh --debug mcb`: a breakpoint stops the hub's tick count, and the run carries on normally after continuing.
