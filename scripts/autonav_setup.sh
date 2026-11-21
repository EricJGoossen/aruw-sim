#!/usr/bin/env bash
# Sets up scripts/run.sh --autonav: project-otto's feature/autonav_framework
# (the 2026-27 ROS 2 stack, sim hooks included) next to the comp branch in
# the submodule, all under build/:
#
#   build/otto-autonav       a worktree of project-otto at REF
#   build/otto-autonav-venv  its Python 3.10 venv
#   aruw-autonav:humble      the ROS 2 Humble image (docker/autonav)
#   build/autonav-ws         its autonav/ ROS packages, built with colcon there
#
#   scripts/autonav_setup.sh [REF]     (default origin/feature/autonav_framework)
#
# scripts/setup.sh --autonav runs it. Re-running fetches and moves the
# worktree to REF's current commit; the submodule's checkout isn't touched.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
ref="${1:-origin/feature/autonav_framework}"
W="$repo/build/otto-autonav" V="$repo/build/otto-autonav-venv"
mkdir -p "$repo/build"

echo "[autonav] otto at $ref -> $W"
git -C project-otto fetch -q origin || echo "[autonav] fetch failed; using what's here" >&2
if [ ! -d "$W" ]; then
  git -C project-otto worktree prune
  git -C project-otto worktree add -q --detach "$W" "$ref"
else
  git -C "$W" reset -q --hard
  git -C "$W" checkout -q --detach "$ref"
fi
git -C "$W" submodule update -q --init --recursive

# otto and its ROS nodes run in the ROS 2 image, so its venv is made there too.
echo "[autonav] image aruw-autonav:humble"
docker build -q -t aruw-autonav:humble docker/autonav >/dev/null
mkdir -p "$repo/build/autonav-ws" "$repo/build/home"
an() {
  docker run --rm --user "$(id -u):$(id -g)" -e HOME="$repo/build/home" \
    -e UV_CACHE_DIR="$repo/build/uv-cache" -e UV_PYTHON_INSTALL_DIR="$repo/build/uv-python" \
    -v "$repo:$repo" -w "$1" aruw-autonav:humble bash -c "$2"
}
echo "[autonav] venv"
an "$W" "UV_PROJECT_ENVIRONMENT='$V' uv sync -q --frozen --python 3.10"
echo "[autonav] ROS packages"
an "$W/autonav" "source /opt/ros/humble/setup.bash && colcon build \
  --build-base $repo/build/autonav-ws/build --install-base $repo/build/autonav-ws/install \
  --event-handlers console_cohesion- >/dev/null"
echo "[autonav] ready: scripts/run.sh --autonav (or --autonav-drive)"
