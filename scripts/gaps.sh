#!/usr/bin/env bash
# Prints a run's sim-gap report and writes <run dir>/gaps.json.
#
#   scripts/gaps.sh [RUN_DIR] [--check]
#
# RUN_DIR defaults to the newest directory under build/run/. Every gap any
# process reported (the hub, the MCB, otto, and the model build) is listed,
# grouped by process; those not in assets/sentry/known_gaps.yaml are marked
# NEW. --check exits 1 if there are any (what smoke.sh uses).
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir=""
args=()
for a in "$@"; do
  case "$a" in
    --*) args+=("$a") ;;
    *) run_dir="$a" ;;
  esac
done
if [ -z "$run_dir" ]; then
  run_dir="$(ls -1dt "$repo"/build/run/*/ 2>/dev/null | head -1)"
  [ -n "$run_dir" ] || { echo "gaps.sh: no runs under build/run/" >&2; exit 1; }
fi
exec python3 "$repo/tools/gap_report.py" "$run_dir" "${args[@]}"
