#!/usr/bin/env python3
"""A run's sim-gap report: every process's gaps, against the known list.

    tools/gap_report.py RUN_DIR [--known assets/sentry/known_gaps.yaml] [--check]

Reads RUN_DIR/gaps/*.jsonl (the hub's, the MCB's, otto's and the model's; see
hw-emulator's gaps.hpp for the format), writes RUN_DIR/gaps.json (all of them,
deduplicated, each marked known or not), and prints them grouped by process.
A gap is known when an entry of the known list matches its process, source
and key (shell-style globs; a missing field matches anything). With --check,
exits 1 if any gap isn't known -- smoke.sh's test.
"""

from __future__ import annotations

import argparse
import fnmatch
import glob
import json
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(run_dir: str) -> list[dict]:
    seen, out = set(), []
    for path in sorted(glob.glob(os.path.join(run_dir, "gaps", "*.jsonl"))):
        with open(path) as f:
            for n, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    g = json.loads(line)
                except json.JSONDecodeError:
                    g = {"process": os.path.basename(path)[:-6], "source": "gap_report", "key": f"bad-line:{n}",
                         "message": f"unparseable gap line in {os.path.basename(path)}:{n}: {line[:120]}", "time": 0}
                k = (g.get("process"), g.get("source"), g.get("key"))
                if k in seen:
                    continue
                seen.add(k)
                out.append(g)
    return out


def matches(entry: dict, gap: dict) -> bool:
    for field in ("process", "source", "key"):
        pat = entry.get(field)
        if pat is not None and not fnmatch.fnmatchcase(str(gap.get(field, "")), str(pat)):
            return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run_dir")
    ap.add_argument("--known", default=os.path.join(ROOT, "assets", "sentry", "known_gaps.yaml"))
    ap.add_argument("--check", action="store_true", help="exit 1 on any gap not in the known list")
    args = ap.parse_args()

    known = []
    if os.path.exists(args.known):
        with open(args.known) as f:
            known = (yaml.safe_load(f) or {}).get("known", [])
    gaps = load(args.run_dir)
    for g in gaps:
        hit = next((k for k in known if matches(k, g)), None)
        g["known"] = hit is not None
        if hit is not None and hit.get("why"):
            g["why"] = hit["why"]
    with open(os.path.join(args.run_dir, "gaps.json"), "w") as f:
        json.dump(gaps, f, indent=1)

    new = [g for g in gaps if not g["known"]]
    order = {"model": 0, "hub": 1, "mcb": 2, "otto": 3}
    print(f"\n[gaps] {len(gaps)} sim gap(s) this run, {len(new)} not in {os.path.relpath(args.known, ROOT)}"
          f"  ({os.path.join(args.run_dir, 'gaps.json')})")
    for proc in sorted({g.get("process", "?") for g in gaps}, key=lambda p: (order.get(p, 9), p)):
        print(f"  {proc}:")
        for g in (g for g in gaps if g.get("process") == proc):
            mark = "   " if g["known"] else "NEW"
            t = g.get("time", 0)
            print(f"   {mark} {g.get('source')}: {g.get('message')}" + (f"  (t={t:g}s)" if t else ""))
    if args.check and new:
        print(f"[gaps] FAIL: {len(new)} gap(s) not in the known list:", file=sys.stderr)
        for g in new:
            print(f"  - process: {g.get('process')}\n    source: {g.get('source')}\n    key: {json.dumps(g.get('key'))}",
                  file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
