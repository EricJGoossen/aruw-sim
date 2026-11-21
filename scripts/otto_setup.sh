#!/usr/bin/env bash
# Sets up one of otto's targets for scripts/run.sh --otto TARGET. Each target
# is a branch of project-otto with its own patch (patches/project-otto/):
#
#   comp     origin/feat/comp-branch-sim-integration, in the project-otto submodule itself
#            (its pinned commit); scripts/build.sh patches it and builds its
#            venv, so there's nothing more to do here. The default.
#   main     origin/main: a worktree in build/otto-main, its venv in
#            build/otto-main-venv (built in the dev container)
#   autonav  origin/feature/autonav_framework, the 2026-27 ROS 2 stack: a
#            worktree in build/otto-autonav, its venv in
#            build/otto-autonav-venv, the aruw-autonav:humble image
#            (docker/autonav), and its ROS packages built in build/autonav-ws
#
#   scripts/otto_setup.sh TARGET [REF]     (REF: another commit or branch)
#
# The submodule's own checkout is never touched by main or autonav. Re-running
# fetches and moves the worktree to REF's current commit. A patch REF already
# has (committed upstream) is skipped; one that neither applies nor is already
# there stops it with the file that failed.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo"
target="${1:-}"
case "$target" in
  comp)
    echo "[otto] comp is the project-otto submodule: scripts/build.sh patches it and builds its venv"
    exit 0 ;;
  main) default_ref=origin/main ;;
  autonav) default_ref=origin/feature/autonav_framework ;;
  *) echo "usage: scripts/otto_setup.sh comp|main|autonav [REF]" >&2; exit 2 ;;
esac
ref="${2:-$default_ref}"
W="$repo/build/otto-$target" V="$repo/build/otto-$target-venv"
patch="$repo/patches/project-otto/$target.patch"
mkdir -p "$repo/build"

echo "[otto] $target: $ref -> $W"
git -C project-otto fetch -q origin || echo "[otto] fetch failed; using what's here" >&2
if [ ! -d "$W" ]; then
  git -C project-otto worktree prune
  git -C project-otto worktree add -q --detach "$W" "$ref"
else
  git -C "$W" reset -q --hard
  git -C "$W" checkout -q --detach "$ref"
fi
if git -C "$W" apply --reverse --check "$patch" 2>/dev/null; then
  echo "[otto] $target: $(basename "$patch") already in $ref"
elif git -C "$W" apply --check "$patch" 2>/dev/null; then
  git -C "$W" apply "$patch"
else
  echo "otto_setup.sh: $(basename "$patch") neither applies to $ref ($(git -C "$W" rev-parse --short HEAD)) nor is already in it:" >&2
  git -C "$W" apply --check "$patch" >&2 || true
  exit 1
fi
git -C "$W" submodule update -q --init --recursive

if [ "$target" = main ]; then
  echo "[otto] main: venv (dev container)"
  "$repo/scripts/dev.sh" bash -c "cd '$W' && UV_PROJECT_ENVIRONMENT='$V' uv sync -q --frozen --python 3.10"
  echo "[otto] ready: scripts/run.sh --otto main"
  exit 0
fi

# autonav: otto and its ROS nodes run in the ROS 2 image, so its venv is made there too.
echo "[otto] autonav: image aruw-autonav:humble"
docker build -q -t aruw-autonav:humble docker/autonav >/dev/null
mkdir -p "$repo/build/autonav-ws" "$repo/build/home"
an() {
  docker run --rm --user "$(id -u):$(id -g)" -e HOME="$repo/build/home" \
    -e UV_CACHE_DIR="$repo/build/uv-cache" -e UV_PYTHON_INSTALL_DIR="$repo/build/uv-python" \
    -v "$repo:$repo" -w "$1" aruw-autonav:humble bash -c "$2"
}
echo "[otto] autonav: venv"
an "$W" "UV_PROJECT_ENVIRONMENT='$V' uv sync -q --frozen --python 3.10"
echo "[otto] autonav: ROS packages"
an "$W/autonav" "source /opt/ros/humble/setup.bash && colcon build \
  --build-base $repo/build/autonav-ws/build --install-base $repo/build/autonav-ws/install \
  --event-handlers console_cohesion- >/dev/null"
echo "[otto] ready: scripts/run.sh --autonav (or --autonav-drive)"
