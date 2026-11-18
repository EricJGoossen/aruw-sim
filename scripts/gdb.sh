#!/usr/bin/env bash
# gdb, from the dev container when the host has none: what .vscode/launch.json
# runs as its debugger ("Attach MCB", "Attach hub"). The checkout is mounted at
# the same path inside, so source paths need no mapping.
if command -v gdb >/dev/null && [ "${ARUW_SIM_CONTAINER_GDB:-0}" != "1" ]; then exec gdb "$@"; fi
exec "$(dirname "${BASH_SOURCE[0]}")/dev.sh" gdb "$@"
