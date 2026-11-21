#!/usr/bin/env bash
# Sets up scripts/run.sh --autonav: the 2026-27 autonav stack (project-otto
# feature/autonav_framework) next to the sim, all under build/ -- the
# submodule's own checkout (the comp branch) isn't touched:
#
#   build/otto-autonav       a worktree of project-otto at REF, with
#                            patches/project-otto-autonav.patch (the sim's
#                            lidar-port and Arducam hooks, for main's layout)
#   build/otto-autonav-venv  its Python 3.10 venv (uv sync --frozen)
#   aruw-autonav:humble      the ROS 2 Humble image (docker/autonav)
#   build/autonav-ws         its autonav/ ROS packages, built with colcon there
#
#   scripts/autonav_setup.sh [REF]     (default origin/feature/autonav_framework)
#
# Re-running it moves the worktree to REF's current commit (fetch first).
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
ref="${1:-origin/feature/autonav_framework}"
W="$repo/build/otto-autonav"
export UV_CACHE_DIR="$repo/build/uv-cache" UV_PYTHON_INSTALL_DIR="$repo/build/uv-python"

echo "[autonav] otto at $ref -> $W"
git -C project-otto fetch -q origin || echo "[autonav] fetch failed; using what's here" >&2
if [ ! -d "$W" ]; then
  git -C project-otto worktree add -q --detach "$W" "$ref"
else
  git -C "$W" checkout -q -- project_otto
  git -C "$W" checkout -q --detach "$ref"
fi
git -C "$W" apply "$repo/patches/project-otto-autonav.patch"
git -C "$W" submodule update -q --init --recursive aruw-lightning-models

echo "[autonav] venv"
(cd "$W" && UV_PROJECT_ENVIRONMENT="$repo/build/otto-autonav-venv" uv sync -q --frozen --python 3.10)

echo "[autonav] image aruw-autonav:humble"
docker build -q -t aruw-autonav:humble docker/autonav >/dev/null

echo "[autonav] ROS packages"
mkdir -p "$repo/build/autonav-ws" "$repo/build/home"
docker run --rm --user "$(id -u):$(id -g)" -e HOME="$repo/build/home" -v "$repo:$repo" -w "$W/autonav" \
  aruw-autonav:humble bash -c "source /opt/ros/humble/setup.bash && colcon build \
    --build-base $repo/build/autonav-ws/build --install-base $repo/build/autonav-ws/install \
    --event-handlers console_cohesion- >/dev/null"
echo "[autonav] ready: scripts/run.sh --autonav"
