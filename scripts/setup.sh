#!/usr/bin/env bash
# Sets up a fresh clone, after
#
#   git clone https://github.com/EricJGoossen/aruw-sim.git && cd aruw-sim
#   git submodule update --init --recursive
#
# so that scripts/run.sh is ready to go:
#
#   scripts/setup.sh [--autonav] [-j N]
#
#   --autonav  also set up the 2026-27 ROS 2 autonav stack (project-otto
#              feature/autonav_framework) for run.sh --autonav: see
#              scripts/autonav_setup.sh
#   -j N       build jobs (default 2; capped by memory, ~3 GB each)
#
# It checks what the host needs (git, Docker, Python 3 with PyYAML), makes
# sure every submodule is checked out, then runs scripts/build.sh (in the dev
# container: the model, hub and Achilles, the firmware, otto's venv) and, with
# --autonav, scripts/autonav_setup.sh. Safe to re-run: every step skips
# what's already done. The first run takes a while (the dev image, the C++
# builds and the Python venvs; tens of minutes).
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
autonav=0 jobs=2
while [ $# -gt 0 ]; do
  case "$1" in
    --autonav) autonav=1; shift ;;
    -j) jobs="$2"; shift 2 ;;
    -j*) jobs="${1#-j}"; shift ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) echo "setup.sh: unknown argument $1 (see --help)" >&2; exit 2 ;;
  esac
done
step() { printf '\n\033[1m[setup] %s\033[0m\n' "$*"; }

step "host"
missing=()
command -v git >/dev/null || missing+=("git")
command -v docker >/dev/null || missing+=("Docker (docker.io, or Docker Desktop with WSL integration)")
command -v python3 >/dev/null && python3 -c "import yaml" 2>/dev/null || missing+=("Python 3 with PyYAML (apt install python3-yaml)")
if [ "${#missing[@]}" -gt 0 ]; then
  printf 'setup.sh: the host needs:\n' >&2; printf '  - %s\n' "${missing[@]}" >&2; exit 1
fi
docker info >/dev/null 2>&1 || { echo "setup.sh: Docker isn't running, or you can't use it (are you in the docker group?)" >&2; exit 1; }
echo "  git, Docker and Python 3 with PyYAML: ok"

step "submodules"
# Anything not checked out yet (a clone without --recursive); ones already
# checked out are left as they are.
if git submodule status --recursive | grep -q '^-'; then
  git submodule update --init --recursive
fi
git submodule status | sed 's/^/  /'

step "build (scripts/build.sh)"
scripts/build.sh -j "$jobs"

if [ "$autonav" = 1 ]; then
  step "autonav (scripts/autonav_setup.sh)"
  scripts/autonav_setup.sh
fi

step "ready"
cat <<EOF
  scripts/run.sh                      the sentry on otto's comp branch (Ctrl-C stops it)
  scripts/run.sh --window --enemy patrol   with its camera window, and an enemy to chase
EOF
[ "$autonav" = 1 ] && echo "  scripts/run.sh --autonav-drive      the ROS 2 autonav stack, with the legacy planner driving"
echo "  scripts/run.sh --help               everything else"
