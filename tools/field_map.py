#!/usr/bin/env python3
"""
Draws the sim's field (assets/field/*.yaml) as a ROS occupancy map, the kind
the 2026-27 autonav stack's AMCL localizes against (project-otto
feature/autonav_framework, autonav/assets/map): a trinary PGM plus its yaml,
of whatever stands in a band of heights above the floor -- 0.10 to 0.40 m,
the band those maps are drawn for and the Odin scan is cut from.

The map is in the field's frame (the MCB's odometry frame), so a pose on it is
a pose in otto's world frame. Outside the perimeter is unknown.

    tools/field_map.py FIELD.yaml OUT_PREFIX [--band 0.10 0.40] [--resolution 0.02]

writes OUT_PREFIX.pgm and OUT_PREFIX.yaml.
"""

import argparse
import os

import numpy as np
import yaml

FREE, OCCUPIED, UNKNOWN = 254, 0, 128
MARGIN = 0.3  # m of unknown around the field, as the autonav maps have


def boxes(field, band):
    """The field's collidable boxes and ramps, as (x0, y0, x1, y1, height_at(x, y) or None)."""
    lo, hi = band
    mx = field["mirror_x"]
    out = []
    for el in field["elements"]:
        if not el.get("collide", True):
            continue
        if "box" in el:
            (x0, y0, z0), (x1, y1, z1) = el["box"]
            if z1 < lo or z0 > hi:
                continue
            shapes = [(x0, y0, x1, y1, None)]
        elif "ramp" in el:
            r = el["ramp"]
            (x0, x1), (y0, y1), rise, up = r["x"], r["y"], r["rise"], r["up"]
            if rise < lo:
                continue
            shapes = [(x0, y0, x1, y1, (rise, up))]
        else:
            continue
        for s in shapes:
            out.append(s)
            if el.get("mirror"):
                x0, y0, x1, y1, ramp = s
                if ramp is not None:
                    ramp = (ramp[0], {"+x": "-x", "-x": "+x"}.get(ramp[1], ramp[1]))
                out.append((2 * mx - x1, y0, 2 * mx - x0, y1, ramp))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("field")
    ap.add_argument("out_prefix")
    ap.add_argument("--band", nargs=2, type=float, default=[0.10, 0.40])
    ap.add_argument("--resolution", type=float, default=0.02)
    args = ap.parse_args()

    field = yaml.safe_load(open(args.field))
    res = args.resolution
    L, W = field["length"], field["width"]
    nx = int(round((L + 2 * MARGIN) / res))
    ny = int(round((W + 2 * MARGIN) / res))
    xs = -MARGIN + (np.arange(nx) + 0.5) * res
    ys = -MARGIN + (np.arange(ny) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)  # rows: y up from the origin
    img = np.full((ny, nx), UNKNOWN, np.uint8)
    img[(X >= 0) & (X <= L) & (Y >= 0) & (Y <= W)] = FREE
    for x0, y0, x1, y1, ramp in boxes(field, args.band):
        inside = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
        if ramp is not None:
            rise, up = ramp
            frac = {"+x": (X - x0) / (x1 - x0), "-x": (x1 - X) / (x1 - x0),
                    "+y": (Y - y0) / (y1 - y0), "-y": (y1 - Y) / (y1 - y0)}[up]
            inside &= rise * frac >= args.band[0]
        img[inside] = OCCUPIED
    # The perimeter's outer faces are outside the field: keep them as walls.
    pgm = np.flipud(img)  # PGM rows run top to bottom
    with open(args.out_prefix + ".pgm", "wb") as f:
        f.write(f"P5\n{nx} {ny}\n255\n".encode())
        f.write(pgm.tobytes())
    meta = {
        "image": os.path.basename(args.out_prefix) + ".pgm",
        "mode": "trinary",
        "resolution": res,
        "origin": [-MARGIN, -MARGIN, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(args.out_prefix + ".yaml", "w") as f:
        f.write(f"# {os.path.basename(args.field)} between {args.band[0]:g} and {args.band[1]:g} m above the floor,\n"
                "# in the field's frame; made by aruw-sim tools/field_map.py.\n")
        yaml.safe_dump(meta, f, sort_keys=False)


if __name__ == "__main__":
    main()
