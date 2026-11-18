#!/usr/bin/env bash
# Builds everything the sentry sim runs, rebuilding only what changed.
#
#   scripts/build.sh [--profile debug|release] [--only STEP[,STEP...]] [-j N]
#
# Steps, in order (--only picks some):
#   patches  apply patches/aruw-mcb.patch, patches/aruw-mcb-fixes.patch (firmware
#            bugs the sim found; ARUW_SIM_NO_FIXES=1 skips it) and
#            patches/project-otto.patch to their submodules, idempotently;
#            refuses loudly if one no longer applies (and isn't already applied)
#   model    tools/doctor_urdf.py, then the Achilles model (Achilles's
#            tools/urdf_to_arow.py + hw-emulator's tools/elements_to_arow.py),
#            when their inputs changed; then tools/check_model.py, always
#   cmake    the hub, Achilles, MuJoCo, the Python module and libaruw_devices
#            (CMake + Ninja + ccache, build/cmake)
#   mcb      the hosted firmware: scons build-sim robot=SENTRY_ACHLYS
#            hwemu=build/hwemu_client
#   otto     otto's venv (uv sync --frozen into build/otto-venv), when
#            uv.lock or pyproject.toml changed
#
# check_model.py stops the build on a mismatch. The three known ones (see
# docs/PLAN.md) are let through for test runs with
#   HWEMU_ACCEPT_MISMATCH=wheelbase_radius.joint_fl_wheel,...
#
# The job count defaults to 2 (Achilles's tests and the hub link take
# several GB per job); -j or ARUW_SIM_JOBS overrides it, and it's capped at
# one job per 3 GB of available memory.
#
# When the host lacks the toolchain (cmake, ninja, g++-11, gtest) and has
# Docker, this re-runs itself in the dev container (scripts/dev.sh).
# ARUW_SIM_NATIVE=1 forces a host build.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"

profile=debug
only=""
jobs="${ARUW_SIM_JOBS:-2}"
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) profile="$2"; shift 2 ;;
    --profile=*) profile="${1#*=}"; shift ;;
    --only) only="$2"; shift 2 ;;
    --only=*) only="${1#*=}"; shift ;;
    -j) jobs="$2"; shift 2 ;;
    -j*) jobs="${1#-j}"; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) echo "build.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done
case "$profile" in debug|release) ;; *) echo "build.sh: --profile is debug or release" >&2; exit 2 ;; esac

have_toolchain() {
  command -v cmake >/dev/null && command -v ninja >/dev/null && command -v g++-11 >/dev/null \
    && [ -e /usr/include/gtest/gtest.h ] && [ -e /usr/include/EGL/egl.h ]
}
if [ -z "${ARUW_SIM_CONTAINER:-}" ] && [ "${ARUW_SIM_NATIVE:-0}" != "1" ] && ! have_toolchain; then
  if command -v docker >/dev/null; then
    exec "$repo/scripts/dev.sh" scripts/build.sh --profile "$profile" ${only:+--only "$only"} -j "$jobs"
  fi
  echo "build.sh: the host lacks cmake/ninja/g++-11/gtest/EGL headers and there's no Docker." >&2
  echo "          Install them (see .devcontainer/Dockerfile) or set ARUW_SIM_NATIVE=1 to try anyway." >&2
  exit 1
fi

# Cap the jobs by memory: ~3 GB each.
avail_kb="$(awk '/MemAvailable/ {print $2}' /proc/meminfo)"
mem_jobs=$(( avail_kb / (3 * 1024 * 1024) )); [ "$mem_jobs" -lt 1 ] && mem_jobs=1
[ "$jobs" -gt "$mem_jobs" ] && { echo "[build] -j $jobs capped to $mem_jobs by available memory"; jobs=$mem_jobs; }

want() { [ -z "$only" ] || [[ ",$only," == *",$1,"* ]]; }
step() { printf '\n\033[1m[build] %s\033[0m\n' "$*"; }
B="$repo/build"
mkdir -p "$B/stamps"

# A stamp holding the hash of a step's inputs; the step reruns when it changes.
inputs_hash() { cat "$@" 2>/dev/null | sha256sum | cut -c1-32; }
stamp_ok() { [ "$(cat "$B/stamps/$1" 2>/dev/null)" = "$2" ]; }
stamp_set() { echo "$2" >"$B/stamps/$1"; }

# --- patches -----------------------------------------------------------------
apply_patch() {
  local sub="$1" patch="$repo/patches/${2:-$1}.patch"
  [ -f "$patch" ] || return 0
  if git -C "$sub" apply --reverse --check "$patch" 2>/dev/null; then
    echo "  $sub: $(basename "$patch") already applied"
  elif git -C "$sub" apply --check "$patch" 2>/dev/null; then
    git -C "$sub" apply "$patch"
    echo "  $sub: applied $(basename "$patch")"
  else
    echo "build.sh: $(basename "$patch") neither applies to $sub nor is already applied:" >&2
    git -C "$sub" apply --check "$patch" >&2 || true
    echo "  Re-export it from a working tree (git -C $sub diff, plus untracked files), or reset $sub." >&2
    exit 1
  fi
}
if want patches; then
  step "patches"
  apply_patch aruw-mcb
  # Firmware bug fixes the sim found -- not emulator interface, so kept apart
  # for upstreaming (docs/PLAN.md). ARUW_SIM_NO_FIXES=1 leaves them out.
  [ "${ARUW_SIM_NO_FIXES:-0}" = "1" ] || apply_patch aruw-mcb aruw-mcb-fixes
  apply_patch project-otto
fi

# --- model -------------------------------------------------------------------
ASSETS="$repo/assets/sentry"
OUT="$B/assets/sentry"
if want model; then
  step "model"
  h="$(inputs_hash tools/doctor_urdf.py "$ASSETS/urdf_doctor.yaml" "$ASSETS"/sentry_description/urdf/*.urdf)"
  if ! stamp_ok doctor "$h" || [ ! -f "$OUT/sentry_mujoco.urdf" ]; then
    # Its gaps are printed and kept, as model gaps, for every run's report.
    python3 tools/doctor_urdf.py "$ASSETS/urdf_doctor.yaml" "$OUT" 2>&1 | tee "$OUT.doctor.log"
    mkdir -p "$OUT"
    python3 - "$OUT.doctor.log" "$OUT/doctor_gaps.jsonl" <<'PY'
import json, sys
with open(sys.argv[2], "w") as out:
    for line in open(sys.argv[1]):
        if line.startswith("[hwemu GAP] "):
            src, _, msg = line[len("[hwemu GAP] "):].strip().partition(": ")
            out.write(json.dumps({"process": "model", "source": src, "key": msg, "message": msg, "time": 0}) + "\n")
PY
    rm -f "$OUT.doctor.log"
    stamp_set doctor "$h"
  else
    echo "  doctored URDFs up to date"
  fi
  h="$(inputs_hash "$OUT/sentry_achilles.urdf" "$ASSETS/devices.yaml" "$ASSETS/sim.yaml" \
        achilles/tools/urdf_to_arow.py hw-emulator/tools/elements_to_arow.py hw-emulator/presets/*.yaml)"
  if ! stamp_ok arow "$h" || [ ! -f "$OUT/achilles/sentry.arow" ]; then
    rm -rf "$OUT/achilles"
    python3 achilles/tools/urdf_to_arow.py "$OUT/sentry_achilles.urdf" "$OUT/achilles" \
        --archetype sentry --gaps "$OUT/achilles/model_gaps.jsonl"
    python3 hw-emulator/tools/elements_to_arow.py --devices "$ASSETS/devices.yaml" --sim "$ASSETS/sim.yaml" \
        --archetype sentry --robot "$OUT/achilles/sentry.arow" --out "$OUT/achilles/parts" \
        --gaps "$OUT/achilles/model_gaps.jsonl"
    stamp_set arow "$h"
  else
    echo "  Achilles model up to date"
  fi
  python3 tools/check_model.py
fi

# --- cmake -------------------------------------------------------------------
CMB="$B/cmake"
if want cmake; then
  step "cmake (hub, Achilles, MuJoCo, Python module, aruw_devices) -j $jobs"
  # The hub stays optimised (with debug info) in both profiles: Achilles at
  # -O0 is too slow to step the sentry, rollers and all, in real time.
  # ARUW_SIM_HUB_O0=1 builds it at -O0 for stepping through the hub itself.
  btype=RelWithDebInfo; [ "${ARUW_SIM_HUB_O0:-0}" = "1" ] && btype=Debug
  launcher=(); command -v ccache >/dev/null && launcher=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
  if [ ! -f "$CMB/build.ninja" ] || [ "$(grep -s '^CMAKE_BUILD_TYPE:' "$CMB/CMakeCache.txt" | cut -d= -f2)" != "$btype" ]; then
    cmake -S . -B "$CMB" -G Ninja -DCMAKE_BUILD_TYPE="$btype" -DHWEMU_WITH_PYTHON=ON \
        -DFETCHCONTENT_BASE_DIR="$B/_deps" "${launcher[@]}"
  fi
  ninja -C "$CMB" -j "$jobs"
fi

# --- mcb ---------------------------------------------------------------------
MCBP="$repo/aruw-mcb/aruw-mcb-project"
if want mcb; then
  step "mcb (scons build-sim robot=SENTRY_ACHLYS profile=$profile) -j $jobs"
  # scons 3.1.2 and the rest of aruw-mcb's Pipfile (lbuild brings jinja2,
  # which modm's templates need) in their own Python 3.11 venv.
  SV="$B/scons-venv"
  h="$(inputs_hash "$MCBP/Pipfile")"
  if ! stamp_ok scons-venv "$h" || [ ! -x "$SV/bin/scons" ]; then
    rm -rf "$SV"
    uv venv -q --python 3.11 "$SV"
    VIRTUAL_ENV="$SV" uv pip install -q scons==3.1.2 lbuild==1.20.0 pyelftools requests
    stamp_set scons-venv "$h"
  fi
  [ -d "$repo/aruw-mcb/segger-rtt/RTT" ] || [ -n "$(ls -A "$repo/aruw-mcb/segger-rtt" 2>/dev/null)" ] \
    || git -C aruw-mcb submodule update --init segger-rtt taproot-scripts
  HC="$B/hwemu_client"
  mkdir -p "$HC"
  [ -f "$CMB/hw-emulator/libhwemu_client.a" ] || { echo "build.sh: no libhwemu_client.a; build the cmake step first" >&2; exit 1; }
  cp -u hw-emulator/client/hwemu_client.h "$CMB/hw-emulator/libhwemu_client.a" "$HC/"
  (
    cd "$MCBP"
    SCONS_LIB_DIR="$(echo "$SV"/lib/python3.11/site-packages/scons)" \
      "$SV/bin/python" "$SV/bin/scons" build-sim robot=SENTRY_ACHLYS profile="$profile" hwemu="$HC" -j "$jobs"
  )
  elf="$MCBP/build/sim-hwemu/scons-$profile/TARGET_SENTRY_ACHLYS/aruw-mcb.elf"
  [ -x "$elf" ] || { echo "build.sh: scons finished but $elf is missing" >&2; exit 1; }
  echo "  $elf"
fi

# --- otto --------------------------------------------------------------------
if want otto; then
  step "otto (uv sync --frozen)"
  OV="$B/otto-venv"
  h="$(inputs_hash project-otto/uv.lock project-otto/pyproject.toml)"
  if ! stamp_ok otto "$h" || [ ! -x "$OV/bin/python" ]; then
    (cd project-otto && UV_PROJECT_ENVIRONMENT="$OV" uv sync --frozen --python 3.10)
    stamp_set otto "$h"
  else
    echo "  otto venv up to date"
  fi
fi

step "done"
