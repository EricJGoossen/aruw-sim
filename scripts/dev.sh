#!/usr/bin/env bash
# Runs a command in the aruw-sim dev container (.devcontainer/Dockerfile),
# building the image first if it's missing.
#
#   scripts/dev.sh                     # a shell
#   scripts/dev.sh scripts/build.sh    # what build.sh does by itself when the
#                                      # host lacks the toolchain
#
# The checkout is mounted at its own host path, so paths in logs, gdb and
# VS Code are the same inside and out. The container runs as you (files it
# writes stay yours), shares the host's network and IPC namespaces (the hub's
# sockets and /dev/shm rings are seen from both sides), and gets the X11 /
# WSLg sockets so `run.sh --window` can open windows. ptrace is allowed, for
# gdb, gdbserver and rr.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${ARUW_SIM_IMAGE:-aruw-sim-dev:latest}"
stamp="$repo/build/.image-stamp"
mkdir -p "$repo/build"
# Rebuild the image when the Dockerfile changes.
want="$(sha256sum "$repo/.devcontainer/Dockerfile" | cut -c1-16)"
if ! docker image inspect "$image" >/dev/null 2>&1 || [ "$(cat "$stamp" 2>/dev/null)" != "$want" ]; then
  echo "[dev.sh] building $image" >&2
  docker build -t "$image" "$repo/.devcontainer" >&2
  echo "$want" >"$stamp"
fi

# stdin stays open (-i) so piped clients work (VS Code drives gdb over it);
# a terminal gets a tty too.
tty_flags=(-i)
if [ -t 0 ] && [ -t 1 ]; then tty_flags=(-it); fi
display_flags=(-e DISPLAY="${DISPLAY:-}")
[ -d /tmp/.X11-unix ] && display_flags+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
if [ -d /mnt/wslg ]; then
  display_flags+=(-v /mnt/wslg:/mnt/wslg -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir)
fi
mkdir -p "$repo/build/home"
# Pass through the knobs the scripts read.
env_flags=()
while IFS='=' read -r name _; do
  case "$name" in ARUW_SIM_*|HWEMU_*|UV_*|PYTHON*) env_flags+=(-e "$name") ;; esac
done < <(env)

exec docker run --rm "${tty_flags[@]}" --init \
  --user "$(id -u):$(id -g)" -e HOME="$repo/build/home" \
  --network host --ipc host --cap-add SYS_PTRACE --security-opt seccomp=unconfined \
  "${display_flags[@]}" "${env_flags[@]}" \
  -e ARUW_SIM_CONTAINER=1 \
  -e UV_CACHE_DIR="$repo/build/uv-cache" -e UV_PYTHON_INSTALL_DIR="$repo/build/uv-python" \
  -e CCACHE_DIR="$repo/build/ccache" \
  -v "$repo:$repo" -w "$repo" \
  "$image" "${@:-bash}"
