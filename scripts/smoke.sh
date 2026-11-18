#!/usr/bin/env bash
# The end-to-end check: the sentry's firmware and otto against the sim, on
# each backend, headless and in lockstep (so the result doesn't depend on how
# fast this machine is).
#
#   scripts/smoke.sh [--seconds N] [--backend achilles|mujoco] [--no-build]
#
# Builds once (with the known model mismatches let through:
# scripts/known_mismatches.sh), then for each backend (default: achilles,
# then mujoco) runs scripts/run.sh --compare --lockstep for N sim seconds
# (default 30) and checks that:
#   - the MCB booted against the hub;
#   - both turret-MCB links connected and their IMUs calibrated;
#   - the chassis rotated in sim truth, and translated by the MCB's own odometry;
#   - otto got odometry, lidar scans and camera frames, and the referee's
#     game status (through the MCB);
#   - every sim gap is in assets/sentry/known_gaps.yaml.
# Exits 1 if any check fails on any backend.
set -uo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
seconds=30 backends=(achilles mujoco) build=1
while [ $# -gt 0 ]; do
  case "$1" in
    --seconds) seconds="$2"; shift 2 ;;
    --backend) backends=("$2"); shift 2 ;;
    --no-build) build=0; shift ;;
    -h|--help) sed -n '2,/^set -uo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) echo "smoke.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done

if [ -z "${ARUW_SIM_CONTAINER:-}" ] && [ "${ARUW_SIM_NATIVE:-0}" != "1" ] \
   && ! { command -v cmake >/dev/null && command -v g++-11 >/dev/null; }; then
  exec "$repo/scripts/dev.sh" scripts/smoke.sh "$@"
fi

# shellcheck source=known_mismatches.sh
source scripts/known_mismatches.sh
if [ "$build" = 1 ]; then
  scripts/build.sh || { echo "[smoke] FAIL: the build failed" >&2; exit 1; }
fi

failed=0
for be in "${backends[@]}"; do
  RT="$repo/build/run/smoke-$be"
  echo -e "\n\033[1m[smoke] $be: $seconds sim s, lockstep, headless\033[0m"
  SMOKE_SECONDS="$seconds" ARUW_SIM_RUNTIME_DIR="$RT" scripts/run.sh --backend "$be" --compare --lockstep --seconds "$seconds" --no-build \
      > "$RT.console.log" 2>&1
  echo "[smoke] run.sh exited $? (console: $RT.console.log)"
  SMOKE_SECONDS="$seconds" python3 - "$RT" "$be" <<'PY' || failed=1
import json, os, re, sys
rt, be = sys.argv[1], sys.argv[2]
def read(name):
    try:
        return open(os.path.join(rt, name), errors="replace").read()
    except FileNotFoundError:
        return ""
hub, mcb, otto = read("logs/hub.log"), read("logs/mcb.log"), read("logs/otto.log")
try:
    cmp = json.load(open(os.path.join(rt, "compare.json")))
except (FileNotFoundError, json.JSONDecodeError):
    cmp = {}
def last(pattern, text, cast=int, default=0):
    m = re.findall(pattern, text)
    return cast(m[-1]) if m else default
checks = []
def check(name, ok, detail=""):
    checks.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
check("the run reached its sim time", cmp.get("sim_time", 0) > 0.95 * float(os.environ.get("SMOKE_SECONDS", "0") or 0),
      f"sim t={cmp.get('sim_time', 0):.1f}s")
check("the MCB booted against the hub", "HWEMU: hardware served by the hw-emulator hub" in mcb)
for t in ("turret_mcb_widow", "turret_mcb_chassis"):
    st = last(rf"{t} \(turret_mcb\): imu_state=(\d+)", hub, default=-1)
    check(f"{t}: link up and IMU calibrated", st == 3, f"imu_state={st}")
yaw = cmp.get("truth_chassis_yaw_travel", 0.0)
check("the chassis rotated (sim truth)", yaw > 0.5, f"{yaw:.2f} rad of yaw")
xy = cmp.get("mcb_chassis_xy_net", 0.0)
check("the chassis translated (MCB odometry)", xy > 0.5,
      f"{xy:.2f} m net, {cmp.get('mcb_chassis_xy_first')} -> {cmp.get('mcb_chassis_xy_last')}")
n = cmp.get("odometry_messages", 0)
check("the MCB sent odometry over the vision link", n > 10, f"{n} messages")
o2m = last(r"otto->mcb \d+ B/(\d+) frames", hub)
check("otto talks back to the MCB", o2m > 0, f"{o2m} frames")
check("otto got the referee's robot identity (via the MCB)", "Received new robot identity" in otto)
check("otto received odometry without rejecting it", "Received new health message" in otto
      and "Failed to register new chassis odometry" not in otto)
lid = [last(rf"{l} \(ld19_lidar\): packets=(\d+)", hub) for l in ("lidar_front", "lidar_back")]
check("the lidars streamed", min(lid) > 0, f"packets {lid}")
check("otto's autonav ran on them", "New attack setpoint" in otto or "setpoint" in otto.lower())
fr = last(r"d435 \(realsense_d435\): frames=(\d+)", hub)
check("the D435 rendered frames", fr > 0, f"{fr} frames")
check("otto processed camera frames", "Average FPS" in otto, "an 'Average FPS' line in otto's log")
ref = last(r"referee \(aruw_referee\): sent=(\d+)", hub)
check("the referee sent game status", ref > 0, f"{ref} messages")
# The hub closes its ports when the run's sim time is up; otto's UART reader
# (cpp_serial) then throws "read: End of file" and aborts. That's the end of
# the run, not a failure: drop it (and the traceback it prints) before looking.
otto_run = otto
m = re.search(r"Exception in thread Thread-\d+ \(_read_forever\):", otto)
if m and "read: End of file" in otto[m.start():]:
    otto_run = otto[:m.start()]
crash = re.search(r"Traceback|Segmentation fault|core dumped|CRITICAL", mcb + otto_run)
check("no crash or traceback in the MCB or otto (before the run's end)", crash is None, crash.group(0) if crash else "")
sys.exit(0 if all(checks) else 1)
PY
  scripts/gaps.sh "$RT" --check > "$RT.gaps.txt" 2>&1 && gap_ok=1 || gap_ok=0
  if [ "$gap_ok" = 1 ]; then
    echo "  [PASS] every sim gap is a known one ($(grep -c '^   ' "$RT.gaps.txt") listed in $RT.gaps.txt)"
  else
    echo "  [FAIL] sim gaps not in assets/sentry/known_gaps.yaml:"; sed -n '/FAIL/,$p' "$RT.gaps.txt" | sed 's/^/    /'
    failed=1
  fi
done
[ "$failed" = 0 ] && echo -e "\n[smoke] PASS" || echo -e "\n[smoke] FAIL"
exit "$failed"
