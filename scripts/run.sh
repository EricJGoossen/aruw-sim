#!/usr/bin/env bash
# Runs the sentry in simulation: the hub, then the MCB firmware, then otto, as
# separate processes against one simulated robot.
#
#   scripts/run.sh [--backend achilles|mujoco] [--window [--window-hz N]] [--lockstep]
#                  [--debug mcb|otto|hub] [--rr] [--seconds N] [--no-build]
#                  [--profile debug|release] [--compare] [--no-otto] [--no-mcb]
#
#   --backend     the physics backend (default achilles)
#   --window      open the sensor windows sim.yaml lists under `windows:` (the
#                 sentry's: the RealSense's image), via DISPLAY (WSLg works);
#                 on Achilles also an overhead view of the field, a camera
#                 12 m up (assets/sentry/overhead_view.arow) that its window
#                 flies: W/A/S/D, E/Q up/down, drag to turn, scroll to dolly.
#                 They draw on the hub's thread, so they cost sim speed: drawn
#                 at 30 Hz with 8 llvmpipe threads, about 10% of the rate
#   --window-hz N how often the windows draw (default 30; fewer: faster sim)
#   --lockstep    the hub ticks only once the MCB and otto have acked the last
#                 tick (sim time stops when either does); default: real time
#   --debug X     run X under a debugger and wait for VS Code to attach
#                 (.vscode/launch.json): mcb and hub under gdbserver
#                 (:2345, :2347), otto under debugpy (:5678). Implies
#                 --lockstep with no ack timeout, so a breakpoint freezes the
#                 whole sim and nothing times out.
#   --rr          record the MCB with rr (replay: rr replay <run dir>/rr/latest-trace)
#   --seconds N   stop after N sim seconds (default: until Ctrl-C)
#   --no-build    skip scripts/build.sh (it's incremental, so normally cheap)
#   --compare     run the hub in-process under scripts/compare.py, which prints
#                 the firmware's belief (its odometry) next to sim truth
#   --no-otto / --no-mcb   leave one out
#   --no-arducam  leave the Arducam out of the sim (its device and its camera):
#                 otto doesn't read it by default (arducam_aruco: False), and
#                 rendering it, 1280x800 at 120 Hz, costs more than the rest of
#                 the hub. Not with --compare.
#
# Headless, the hub's camera rendering uses one llvmpipe thread (LP_NUM_THREADS=1): with
# the hub, the firmware and otto all busy, more render threads only fight
# over cores. Set LP_NUM_THREADS to override.
#
# Logs go to <run dir>/logs/<process>.log, and the console shows every line
# prefixed with its process. One Ctrl-C stops all three; if one exits, the
# others are stopped. At the end, scripts/gaps.sh prints the run's gap report
# (every process's, plus the model's) and writes <run dir>/gaps.json.
#
# The run dir is build/run/<backend> (ARUW_SIM_RUNTIME_DIR overrides it).
# check_model.py stops the build on a model mismatch; for test runs, the known
# ones are let through with HWEMU_ACCEPT_MISMATCH (scripts/known_mismatches.sh).
#
# When the host lacks the toolchain, this re-runs itself in the dev container
# (scripts/dev.sh), like build.sh.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"

backend=achilles window=0 lockstep=0 debug="" rr=0 seconds="" build=1 profile=debug compare=0
want_otto=1 want_mcb=1 no_arducam=0 window_hz=""
orig_args=("$@")
while [ $# -gt 0 ]; do
  case "$1" in
    --backend) backend="$2"; shift 2 ;;
    --backend=*) backend="${1#*=}"; shift ;;
    --window) window=1; shift ;;
    --headless) window=0; shift ;;
    --window-hz) window_hz="$2"; shift 2 ;;
    --window-hz=*) window_hz="${1#*=}"; shift ;;
    --lockstep) lockstep=1; shift ;;
    --debug) debug="$2"; shift 2 ;;
    --debug=*) debug="${1#*=}"; shift ;;
    --rr) rr=1; shift ;;
    --seconds) seconds="$2"; shift 2 ;;
    --seconds=*) seconds="${1#*=}"; shift ;;
    --no-build) build=0; shift ;;
    --profile) profile="$2"; shift 2 ;;
    --profile=*) profile="${1#*=}"; shift ;;
    --compare) compare=1; shift ;;
    --no-otto) want_otto=0; shift ;;
    --no-mcb) want_mcb=0; shift ;;
    --no-arducam) no_arducam=1; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) echo "run.sh: unknown argument $1 (see --help)" >&2; exit 2 ;;
  esac
done
case "$backend" in achilles|mujoco) ;; *) echo "run.sh: --backend is achilles or mujoco" >&2; exit 2 ;; esac
case "$debug" in ""|mcb|otto|hub) ;; *) echo "run.sh: --debug is mcb, otto or hub" >&2; exit 2 ;; esac
[ -n "$debug" ] && lockstep=1

if [ -z "${ARUW_SIM_CONTAINER:-}" ] && [ "${ARUW_SIM_NATIVE:-0}" != "1" ] \
   && ! { command -v cmake >/dev/null && command -v g++-11 >/dev/null && command -v gdbserver >/dev/null; }; then
  command -v docker >/dev/null || { echo "run.sh: no toolchain and no Docker (see .devcontainer/Dockerfile)" >&2; exit 1; }
  exec "$repo/scripts/dev.sh" scripts/run.sh "${orig_args[@]}"
fi

if [ "$build" = 1 ]; then
  if ! scripts/build.sh --profile "$profile"; then
    echo "run.sh: the build failed; nothing started." >&2
    exit 1
  fi
fi

B="$repo/build"
RT="${ARUW_SIM_RUNTIME_DIR:-$B/run/$backend}"
HUB="$B/cmake/hw-emulator/hwemu"
ELF="$repo/aruw-mcb/aruw-mcb-project/build/sim-hwemu/scons-$profile/TARGET_SENTRY_ACHLYS/aruw-mcb.elf"
OV="$B/otto-venv"
for f in "$HUB" "$ELF" "$OV/bin/python"; do
  [ -e "$f" ] || { echo "run.sh: $f is missing; run scripts/build.sh" >&2; exit 1; }
done

rm -rf "$RT"
mkdir -p "$RT/logs" "$RT/gaps"
# The model's gaps (the doctor's, and Achilles's converters') are this run's too.
{
  cat "$B/assets/sentry/doctor_gaps.jsonl" 2>/dev/null || true
  [ "$backend" = achilles ] && cat "$B/assets/sentry/achilles/model_gaps.jsonl" 2>/dev/null || true
} >"$RT/gaps/model.jsonl"

export HWEMU_RUNTIME_DIR="$RT"
participants=()
[ "$want_mcb" = 1 ] && participants+=(mcb)
[ "$want_otto" = 1 ] && participants+=(otto)
lock_env=()
if [ "$lockstep" = 1 ]; then lock_env=(HWEMU_LOCKSTEP=1); fi

sim_yaml=assets/sentry/sim.yaml devices_yaml=assets/sentry/devices.yaml
overhead=0
[ "$window" = 1 ] && [ "$backend" = achilles ] && overhead=1
if [ "$no_arducam" = 1 ] || [ "$overhead" = 1 ]; then
  # This run's sim.yaml and devices.yaml, and a copy of the Achilles model:
  # without the Arducam's device and camera (--no-arducam), and with the
  # overhead camera and its window (--window on Achilles).
  python3 - "$repo" "$B" "$RT/model" "$no_arducam" "$overhead" <<'PY'
import os, shutil, sys, yaml
repo, build, out, no_arducam, overhead = sys.argv[1:6]
assets = f"{repo}/assets/sentry"
os.makedirs(out, exist_ok=True)
dev = yaml.safe_load(open(f"{assets}/devices.yaml"))
dev["plugins"] = [os.path.normpath(os.path.join(assets, p)) for p in dev.get("plugins", [])]
if no_arducam == "1":
    dev["devices"] = [d for d in dev["devices"] if d.get("type") != "arducam"]
yaml.safe_dump(dev, open(f"{out}/devices.yaml", "w"), sort_keys=False)
sim = yaml.safe_load(open(f"{assets}/sim.yaml"))
for k in ("urdf", "arow", "achilles_config"):
    sim["models"][k] = os.path.normpath(os.path.join(assets, sim["models"][k]))
model = f"{out}/achilles"
shutil.copytree(os.path.dirname(sim["models"]["arow"]), model)
arow = os.path.join(model, os.path.basename(sim["models"]["arow"]))
text = open(arow).read()
if no_arducam == "1":
    text = text.replace("  - parts/arducam_image.arow\n", "").replace("- parts/arducam_image.arow\n", "")
if overhead == "1":
    shutil.copy(f"{assets}/overhead_mount.arow", f"{model}/overhead_mount.arow")
    shutil.copy(f"{assets}/overhead_view.arow", f"{model}/parts/overhead_view.arow")
    head, sep, rest = text.partition("includes:\n")
    indent = rest[:len(rest) - len(rest.lstrip(" "))]
    text = head + sep + f"{indent}- overhead_mount.arow\n{indent}- parts/overhead_view.arow\n" + rest
    sim["windows"] = [{"sensor": "overhead_view", "view": "image", "title": "Field (overhead)", "fly": True}] + \
        list(sim.get("windows") or [])
open(arow, "w").write(text)
sim["models"]["arow"] = arow
yaml.safe_dump(sim, open(f"{out}/sim.yaml", "w"), sort_keys=False)
PY
  sim_yaml="$RT/model/sim.yaml" devices_yaml="$RT/model/devices.yaml"
fi
hub_args=(--sim "$sim_yaml" --devices "$devices_yaml" --backend "$backend"
          --runtime-dir "$RT" --status 5)
if [ "$window" = 1 ]; then
  hub_args+=(--window --draw-period "$(awk -v hz="${window_hz:-30}" 'BEGIN { print 1 / hz }')")
else
  hub_args+=(--headless)
fi
[ -n "$seconds" ] && hub_args+=(--seconds "$seconds")
if [ "$lockstep" = 1 ]; then
  hub_args+=(--lockstep)
  [ ${#participants[@]} -gt 0 ] && hub_args+=(--expect "$(IFS=,; echo "${participants[*]}")")
  [ -n "$debug" ] && hub_args+=(--ack-timeout 0)
fi

pids=()
names=()
# start NAME CMD...: runs CMD with its output in logs/NAME.log and, prefixed,
# on the console.
start() {
  local name="$1"; shift
  local color
  case "$name" in hub) color=36 ;; mcb) color=33 ;; otto) color=35 ;; *) color=37 ;; esac
  "$@" > >(tee -a "$RT/logs/$name.log" | sed -u "s/^/$(printf '\033[%sm' "$color")[$name]$(printf '\033[0m') /") 2>&1 &
  pids+=($!)
  names+=("$name")
  echo "[run] $name: pid $! ($*)" >&2
}

stop_all() {
  trap - INT TERM
  for p in "${pids[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  for _ in $(seq 50); do
    local alive=0
    for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ "$alive" = 0 ] && break
    sleep 0.1
  done
  for p in "${pids[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap 'echo "[run] stopping" >&2; stop_all; scripts/gaps.sh "$RT" || true; exit 130' INT TERM

# --- hub ---------------------------------------------------------------------
# One llvmpipe thread headless; the sensor windows render on the hub's
# thread too, and need more (see --window).
lp_threads="${LP_NUM_THREADS:-$([ "$window" = 1 ] && echo 8 || echo 1)}"
if [ "$compare" = 1 ]; then
  # The same hub, in-process, with the comparison printed beside it.
  cmp_args=(--backend "$backend" --runtime-dir "$RT")
  [ -n "$seconds" ] && cmp_args+=(--seconds "$seconds")
  if [ "$lockstep" = 1 ]; then
    cmp_args+=(--lockstep)
    [ ${#participants[@]} -gt 0 ] && cmp_args+=(--expect "$(IFS=,; echo "${participants[*]}")")
    [ -n "$debug" ] && cmp_args+=(--ack-timeout 0)
  fi
  start hub python3 -u scripts/compare.py "${cmp_args[@]}"
elif [ "$debug" = hub ]; then
  echo "[run] hub: waiting for a debugger on :2347 (VS Code: \"Attach hub\")" >&2
  start hub env LP_NUM_THREADS="$lp_threads" gdbserver --once :2347 "$HUB" "${hub_args[@]}"
else
  start hub env LP_NUM_THREADS="$lp_threads" "$HUB" "${hub_args[@]}"
fi

# Wait for the hub's ports.
waited=0
until [ -e "$RT/ready" ]; do
  kill -0 "${pids[0]}" 2>/dev/null || { echo "run.sh: the hub exited before it was ready (see $RT/logs/hub.log)" >&2; stop_all; scripts/gaps.sh "$RT" || true; exit 1; }
  sleep 0.1
  waited=$((waited + 1))
  if [ "$debug" != hub ] && [ "$waited" -gt 1200 ]; then
    echo "run.sh: the hub wasn't ready after 120 s" >&2; stop_all; exit 1
  fi
done

# --- mcb ---------------------------------------------------------------------
if [ "$want_mcb" = 1 ]; then
  mcb_cmd=("$ELF")
  # ARUW_SIM_MCB_WRAPPER runs the firmware under something, e.g. a backtrace
  # on a crash: ARUW_SIM_MCB_WRAPPER="gdb -q -batch -ex run -ex bt --args"
  if [ -n "${ARUW_SIM_MCB_WRAPPER:-}" ]; then
    read -r -a wrapper <<<"$ARUW_SIM_MCB_WRAPPER"
    mcb_cmd=("${wrapper[@]}" "$ELF")
  fi
  if [ "$debug" = mcb ]; then
    echo "[run] mcb: waiting for a debugger on :2345 (VS Code: \"Attach MCB\")" >&2
    mcb_cmd=(gdbserver --once :2345 "$ELF")
  elif [ "$rr" = 1 ]; then
    mkdir -p "$RT/rr"
    mcb_cmd=(env _RR_TRACE_DIR="$RT/rr" rr record "$ELF")
  fi
  start mcb env HWEMU_PROCESS=mcb "${lock_env[@]}" "${mcb_cmd[@]}"
fi

# --- otto --------------------------------------------------------------------
if [ "$want_otto" = 1 ]; then
  # otto's sim overlay, with the run dir filled in.
  python3 - "$repo/assets/sentry/otto_sim.yaml" "$RT/otto_sim.yaml" <<'PY'
import os, sys
text = open(sys.argv[1]).read()
open(sys.argv[2], "w").write(os.path.expandvars(text))
PY
  # otto writes some logs relative to its working directory (aruco_logs/), so
  # it runs in the run dir, with its assets/ (the detector weights) linked in.
  mkdir -p "$RT/otto"
  ln -sfn "$repo/project-otto/assets" "$RT/otto/assets"
  pypath="$repo/shims:$repo/hw-emulator/python/shims:$repo/hw-emulator/python:$repo/project-otto"
  otto_cmd=("$OV/bin/python" "$repo/project-otto/project_otto/main.py")
  if [ "$debug" = otto ]; then
    # debugpy lives outside otto's locked venv.
    DP="$B/debugpy"
    [ -d "$DP/debugpy" ] || uv pip install -q --python "$OV/bin/python" --target "$DP" debugpy
    pypath="$pypath:$DP"
    echo "[run] otto: waiting for a debugger on :5678 (VS Code: \"Attach otto\")" >&2
    otto_cmd=("$OV/bin/python" -m debugpy --listen 0.0.0.0:5678 --wait-for-client "$repo/project-otto/project_otto/main.py")
  fi
  start otto env -C "$RT/otto" HWEMU_PROCESS=otto PYTHONPATH="$pypath" "${lock_env[@]}" "${otto_cmd[@]}" \
    "$RT/otto_sim.yaml" "$repo/project-otto/sentry_name_config.yaml" "$repo/project-otto/base_config.yaml"
fi

# --- supervise ---------------------------------------------------------------
status=0
wait -n "${pids[@]}" || status=$?
for i in "${!pids[@]}"; do
  if ! kill -0 "${pids[$i]}" 2>/dev/null; then
    echo "[run] ${names[$i]} exited (status $status); stopping the rest" >&2
    break
  fi
done
stop_all
scripts/gaps.sh "$RT" || true
exit "$status"
