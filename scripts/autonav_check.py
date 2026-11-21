#!/usr/bin/env python3
"""Checks a run's auto-nav against ground truth, and two runs against each other.

    scripts/autonav_check.py RUN_DIR [RUN_DIR ...]

A run dir is what scripts/run.sh leaves (build/run/<backend>): truth.jsonl,
the hub's truth log (sim.yaml truth_log), and vision.jsonl, every message
on the MCB-otto link. For each run it prints:

- the chassis's true motion: distance, speed, and whether it ever left the
  floor (ramps), tipped, or came within a chassis radius of a wall;
- the MCB's odometry against truth: position and heading error over time,
  and drift per metre travelled -- otto places everything (its lidar map,
  its paths) with this odometry. Its "chassis" is the turret major's pose
  (SentryTransformAdapter::getVisionWorldToChassis), so the heading is
  checked against the chassis's yaw plus joint_major's angle (sim.yaml
  truth_log encoders: [major_yaw_encoder]) when the log has it;
- path following: how far the true chassis is from otto's latest
  auto_nav_setpoint path (cross-track error), and which destinations it
  reached (within 0.25 m).

With two runs it prints them side by side. Standard library only.
"""
from __future__ import annotations

import bisect
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHASSIS_RADIUS = 0.29  # m: the chassis's half-width across its flats (chassis.stl)
REACHED = 0.25  # m


def read_jsonl(path):
    out = []
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def roll_pitch(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def field_walls():
    """The field's collidable boxes taller than 5 cm, as 2-D rectangles
    (x0, y0, x1, y1), mirrored ones included (assets/field)."""
    try:
        import yaml  # optional: only for the wall clearance
    except ImportError:
        return []
    with open(os.path.join(ROOT, "assets", "field", "arc2026_3v3.yaml")) as f:
        field = yaml.safe_load(f)
    mx = field["mirror_x"]
    rects = []
    for el in field["elements"]:
        if "box" not in el or not el.get("collide", True):
            continue
        (x0, y0, z0), (x1, y1, z1) = el["box"]
        if z1 - max(z0, 0.0) < 0.05 or z1 <= 0.05:
            continue
        rects.append((x0, y0, x1, y1, el["name"]))
        if el.get("mirror"):
            rects.append((2 * mx - x1, y0, 2 * mx - x0, y1, el["name"] + "_blue"))
    return rects


def rect_distance(px, py, r):
    x0, y0, x1, y1, _ = r
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def segment_distance(px, py, a, b):
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def percentile(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(p / 100 * len(s)))]


def analyse(run):
    truth = read_jsonl(os.path.join(run, "truth.jsonl"))
    vision = read_jsonl(os.path.join(run, "vision.jsonl"))
    if not truth:
        raise SystemExit(f"autonav_check: {run} has no truth.jsonl (sim.yaml truth_log)")
    tt = [r["t"] for r in truth]

    def truth_at(t):
        i = min(max(bisect.bisect_left(tt, t), 0), len(tt) - 1)
        return truth[i]

    res = {"run": run, "sim_time": tt[-1]}
    # -- true motion
    dist, vmax, zmax, tilt = 0.0, 0.0, -1.0, 0.0
    for a, b in zip(truth, truth[1:]):
        d = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        dist += d
        vmax = max(vmax, d / max(b["t"] - a["t"], 1e-9))
        zmax = max(zmax, b["z"])
        tilt = max(tilt, max(abs(v) for v in roll_pitch(b["q"])))
    walls = field_walls()
    closest = (float("inf"), None)
    for r in truth[::5]:
        for w in walls:
            d = rect_distance(r["x"], r["y"], w)
            if d < closest[0]:
                closest = (d, w[4])
    res.update(distance=dist, vmax=vmax, zmax=zmax, tilt_deg=math.degrees(tilt),
               wall_clearance=closest[0] - CHASSIS_RADIUS if walls else float("nan"), nearest_wall=closest[1],
               start=(truth[0]["x"], truth[0]["y"]), end=(truth[-1]["x"], truth[-1]["y"], truth[-1]["yaw"]))

    # -- MCB odometry vs truth
    odo = [(r["t"], r["msg"]["chassis"]) for r in vision
           if r.get("dir") == "mcb->otto" and r["msg"].get("type") == "odometry" and "chassis" in r["msg"]]
    perr, yerr, series = [], [], []
    for t, ch in odo:
        tr = truth_at(t)
        e = math.hypot(ch[0] - tr["x"], ch[1] - tr["y"])
        perr.append(e)
        yerr.append(abs(wrap(ch[5] - tr["yaw"] - tr.get("encoders", {}).get("major_yaw_encoder", 0.0))))
        series.append((t, e))
    res.update(odo_n=len(odo), odo_err_final=perr[-1] if perr else float("nan"),
               odo_err_max=max(perr) if perr else float("nan"),
               odo_yaw_err_max_deg=math.degrees(max(yerr)) if yerr else float("nan"),
               odo_yaw_err_median_deg=math.degrees(percentile(yerr, 50)) if yerr else float("nan"),
               odo_yaw_vs="turret major" if "encoders" in truth[0] else "chassis",
               odo_drift_per_m=(perr[-1] / dist) if perr and dist > 0.5 else float("nan"),
               odo_err_at=list({round(t / 20) * 20: e for t, e in reversed(series)
                                if abs(t - round(t / 20) * 20) < 0.003}.items())[::-1][:7])

    # -- path following
    paths = [(r["t"], r["msg"]["points"], r["msg"].get("speed")) for r in vision
             if r.get("dir") == "otto->mcb" and r["msg"].get("type") == "auto_nav_setpoint" and r["msg"].get("points")]
    cross, reached, dests = [], set(), []
    pi = 0
    for r in truth[::10]:
        while pi + 1 < len(paths) and paths[pi + 1][0] <= r["t"]:
            pi += 1
        if not paths or paths[pi][0] > r["t"]:
            continue
        pts = paths[pi][1]
        d = min(segment_distance(r["x"], r["y"], a, b) for a, b in zip(pts, pts[1:])) if len(pts) > 1 else \
            math.hypot(r["x"] - pts[0][0], r["y"] - pts[0][1])
        cross.append(d)
        dest = tuple(round(v, 2) for v in pts[-1])
        if math.hypot(r["x"] - pts[-1][0], r["y"] - pts[-1][1]) < REACHED:
            reached.add(dest)
    for _, pts, _ in paths:
        dest = tuple(round(v, 2) for v in pts[-1])
        if not dests or dests[-1] != dest:
            dests.append(dest)
    res.update(paths=len(paths), destinations=dests, reached=[d for d in dests if d in reached],
               n_dest=len(set(dests)), n_reached=len(reached),
               cross_median=percentile(cross, 50), cross_p95=percentile(cross, 95),
               cross_max=max(cross) if cross else float("nan"))

    # -- the ROS 2 localization (run.sh --autonav): its pose of the turret major vs truth
    loc_path = os.path.join(run, "autonav", "localization.jsonl")
    loc = read_jsonl(loc_path) if os.path.exists(loc_path) else []
    lerr, lyaw = [], []
    for r in loc:
        tr = truth_at(r["t"])
        lerr.append(math.hypot(r["x"] - tr["x"], r["y"] - tr["y"]))
        lyaw.append(abs(wrap(r["yaw"] - tr["yaw"] - tr.get("encoders", {}).get("major_yaw_encoder", 0.0))))
    res.update(loc_n=len(loc), loc_median=percentile(lerr, 50), loc_p95=percentile(lerr, 95),
               loc_max=max(lerr) if lerr else float("nan"),
               loc_yaw_median_deg=math.degrees(percentile(lyaw, 50)) if lyaw else float("nan"))

    # -- the Odin's own pose (run.sh --autonav): its output vs the truth it was made from
    odin_path = os.path.join(run, "autonav", "odin_pose.jsonl")
    odin = read_jsonl(odin_path) if os.path.exists(odin_path) else []
    oerr = [math.hypot(r["x"] - r["true"][0], r["y"] - r["true"][1]) for r in odin]
    oyaw = [abs(wrap(r["yaw"] - r["true"][3])) for r in odin]
    ev_path = os.path.join(run, "autonav", "odin_events.jsonl")
    events = read_jsonl(ev_path) if os.path.exists(ev_path) else []
    fixes = [e for e in events if e.get("event") in ("relocalized", "wrong fix")]
    res.update(odin_n=len(odin), odin_first_fix=odin[0]["t"] if odin else float("nan"),
               odin_rms=math.sqrt(sum(e * e for e in oerr) / len(oerr)) if oerr else float("nan"),
               odin_max=max(oerr) if oerr else float("nan"),
               odin_yaw_rms_deg=math.degrees(math.sqrt(sum(e * e for e in oyaw) / len(oyaw))) if oyaw else float("nan"),
               odin_fixes=", ".join(f"{e['event']} t={e['t']:.1f}" for e in fixes) or "none")
    return res


ROWS = [
    ("sim time (s)", "sim_time", "{:.1f}"),
    ("distance driven, truth (m)", "distance", "{:.2f}"),
    ("top speed, truth (m/s)", "vmax", "{:.2f}"),
    ("highest footprint z (m)", "zmax", "{:.3f}"),
    ("worst roll/pitch (deg)", "tilt_deg", "{:.1f}"),
    ("closest to a wall, chassis edge (m)", "wall_clearance", "{:.2f}"),
    ("  that wall", "nearest_wall", "{}"),
    ("end pose, truth (x, y, yaw)", "end", "({:.2f}, {:.2f}, {:+.2f})"),
    ("MCB odometry messages", "odo_n", "{}"),
    ("odometry error at the end (m)", "odo_err_final", "{:.3f}"),
    ("odometry error, worst (m)", "odo_err_max", "{:.3f}"),
    ("odometry heading checked against", "odo_yaw_vs", "{}"),
    ("odometry heading error, median (deg)", "odo_yaw_err_median_deg", "{:.2f}"),
    ("odometry heading error, worst (deg)", "odo_yaw_err_max_deg", "{:.2f}"),
    ("odometry drift per metre driven", "odo_drift_per_m", "{:.1%}"),
    ("auto-nav paths sent", "paths", "{}"),
    ("cross-track error, median (m)", "cross_median", "{:.3f}"),
    ("cross-track error, 95th pct (m)", "cross_p95", "{:.3f}"),
    ("cross-track error, worst (m)", "cross_max", "{:.3f}"),
    ("distinct destinations", "n_dest", "{}"),
    ("destinations reached", "n_reached", "{}"),
    ("localization poses (--autonav)", "loc_n", "{}"),
    ("localization error, median (m)", "loc_median", "{:.3f}"),
    ("localization error, 95th pct (m)", "loc_p95", "{:.3f}"),
    ("localization error, worst (m)", "loc_max", "{:.3f}"),
    ("localization heading error, median (deg)", "loc_yaw_median_deg", "{:.2f}"),
    ("Odin poses (--autonav)", "odin_n", "{}"),
    ("Odin first pose (s)", "odin_first_fix", "{:.1f}"),
    ("Odin pose error, rms (m)", "odin_rms", "{:.3f}"),
    ("Odin pose error, worst (m)", "odin_max", "{:.3f}"),
    ("Odin heading error, rms (deg)", "odin_yaw_rms_deg", "{:.2f}"),
    ("Odin fixes", "odin_fixes", "{}"),
]


def fmt(spec, v):
    try:
        if isinstance(v, tuple):
            return spec.format(*v)
        return spec.format(v)
    except (TypeError, ValueError, IndexError):
        return str(v)


def main():
    runs = [analyse(r) for r in sys.argv[1:]]
    if not runs:
        print(__doc__)
        return 2
    names = [os.path.basename(os.path.normpath(r["run"])) for r in runs]
    w = 38
    print(f"{'':{w}s}" + "".join(f"{n:>26s}" for n in names))
    for label, key, spec in ROWS:
        print(f"{label:{w}s}" + "".join(f"{fmt(spec, r[key]):>26s}" for r in runs))
    for r, n in zip(runs, names):
        if r["odo_err_at"]:
            print(f"{n}: odometry error every 20 s: " + ", ".join(f"t={t:.0f} {e:.3f} m" for t, e in r["odo_err_at"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
