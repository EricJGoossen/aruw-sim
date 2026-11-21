#!/usr/bin/env python3
"""Builds the URDFs the sim loads from the vendored sentry URDF.

    tools/doctor_urdf.py assets/sentry/urdf_doctor.yaml build/assets/sentry

The vendored URDFs (assets/sentry/sentry_description/urdf) stay unchanged.
This reads them and urdf_doctor.yaml, and writes:
- sentry_mujoco.urdf, from the coarse URDF (convex-hull collision meshes);
- sentry_achilles.urdf, from the primitives URDF.

It adds:
- a floor and a floating base;
- the sensor frames;
- passive omni-wheel rollers, replacing the solid wheels' collision;
- the flywheel and agitator links;
- armor plates with light bars.

Every number comes from the YAML; nothing here reads the robot code
(tools/check_model.py does, to check the YAML against it).

Anything it can't express is printed as a sim gap, never dropped silently.
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml


def gap(message: str) -> None:
    print(f"[hwemu GAP] doctor_urdf: {message}", file=sys.stderr)


def fmt(values) -> str:
    return " ".join(f"{v:.9g}" for v in values)


def norm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def rpy_from_matrix(m):
    """URDF rpy (fixed x, y, z) of a rotation matrix given as rows."""
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(m[2][0]) < 0.999999:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    return [roll, pitch, yaw]


def frame_with_z(z):
    """A rotation (rows) whose z axis is `z`."""
    z = norm(z)
    helper = [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0]
    x = norm(cross(helper, z))
    y = cross(z, x)
    return [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]


def sub(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def inertial(link, mass, inertia=(1e-6, 1e-6, 1e-6), xyz=(0, 0, 0)):
    i = sub(link, "inertial")
    sub(i, "origin", xyz=fmt(xyz), rpy="0 0 0")
    sub(i, "mass", value=f"{mass:.9g}")
    sub(i, "inertia", ixx=inertia[0], iyy=inertia[1], izz=inertia[2], ixy=0, ixz=0, iyz=0)


def box_geom(parent, tag, size, xyz=(0, 0, 0), rpy=(0, 0, 0), rgba=None, name=None):
    g = sub(parent, tag, **({"name": f"{name}_{'vis' if tag == 'visual' else 'col'}"} if name else {}))
    sub(g, "origin", xyz=fmt(xyz), rpy=fmt(rpy))
    sub(sub(g, "geometry"), "box", size=fmt(size))
    if rgba is not None and tag == "visual":
        m = sub(g, "material", name=f"{name or 'm'}_mat")
        sub(m, "color", rgba=fmt(rgba))
    return g


def octagon_boxes(layer):
    """An octagonal prism -- |x| <= hx, |y| <= hy, |x| + |y| <= chamfer, z in
    [z0, z1] -- as six boxes: two crossing ones for the flats, and one per
    cut corner, turned 45 degrees, covering the triangle the crossing ones
    leave there. (size, xyz, rpy) each."""
    hx, hy = layer["half"]
    c = layer["chamfer"]
    z0, z1 = layer["z"]
    if not (max(hx, hy) < c < hx + hy):
        raise SystemExit(f"doctor_urdf: octagon layer {layer}: needs max(half) < chamfer < sum(half)")
    zc, h = (z0 + z1) / 2, z1 - z0
    boxes = [([2 * hx, 2 * (c - hx), h], [0, 0, zc], [0, 0, 0]),
             ([2 * (c - hy), 2 * hy, h], [0, 0, zc], [0, 0, 0])]
    # The corner left uncovered: right angle at (c - hy, c - hx), hypotenuse
    # the chamfer from (hx, c - hx) to (c - hy, hy).
    length = math.hypot(hx - (c - hy), hy - (c - hx))
    depth = (c - ((c - hy) + (c - hx))) / math.sqrt(2)
    mid = [(hx + c - hy) / 2, (c - hx + hy) / 2]
    for sx in (1, -1):
        for sy in (1, -1):
            n = [sx / math.sqrt(2), sy / math.sqrt(2)]
            centre = [sx * mid[0] - n[0] * depth / 2, sy * mid[1] - n[1] * depth / 2, zc]
            boxes.append(([depth, length, h], centre, [0, 0, math.atan2(sy, sx)]))
    return boxes


RAMP_THICKNESS = 0.04  # the slab a ramp is modelled as; its top face is the slope


def ramp_box(x, y, rise, up):
    """A wedge from the floor up to `rise`, rising toward `up` (+x, -x, +y,
    -y), as a slab: (size, xyz, rpy) of a box whose top face is the slope.
    Its low end dips below the floor and its high end into whatever it
    climbs to; neither touches anything there (the field doesn't collide
    with itself)."""
    t = RAMP_THICKNESS
    along_x = up in ("+x", "-x")
    run = (x[1] - x[0]) if along_x else (y[1] - y[0])
    a = math.atan2(rise, run)
    s, c = math.sin(a), math.cos(a)
    mid = [(x[0] + x[1]) / 2, (y[0] + y[1]) / 2, rise / 2]  # the slope's centre
    # The slope's upward normal, and the rotation turning the slab's long
    # axis onto the slope (URDF rpy: R_y(p) takes x to (cos p, 0, -sin p);
    # R_x(r) takes y to (0, cos r, sin r)).
    normal, rpy = {
        "+x": ([-s, 0.0, c], [0.0, -a, 0.0]),
        "-x": ([s, 0.0, c], [0.0, a, 0.0]),
        "+y": ([0.0, -s, c], [a, 0.0, 0.0]),
        "-y": ([0.0, s, c], [-a, 0.0, 0.0]),
    }[up]
    length = math.hypot(run, rise)
    size = [length, y[1] - y[0], t] if along_x else [x[1] - x[0], length, t]
    xyz = [m - n * t / 2 for m, n in zip(mid, normal)]
    return size, xyz, rpy


def build_field(world_link, field: dict) -> None:
    """Adds the field (assets/field/*.yaml) to the world link: a collision
    and a visual per element, a visual only for markings."""
    colors = field["colors"]
    mx = field["mirror_x"]

    def add(name, size, xyz, rpy, color, collide):
        if collide:
            box_geom(world_link, "collision", size, xyz=xyz, rpy=rpy, name=f"field_{name}")
        box_geom(world_link, "visual", size, xyz=xyz, rpy=rpy, rgba=colors[color], name=f"field_{name}")

    def add_box(name, lo, hi, color, collide):
        size = [h - l for l, h in zip(lo, hi)]
        if min(size) <= 0:
            raise SystemExit(f"doctor_urdf: field element {name}: empty box {lo} {hi}")
        add(name, size, [(l + h) / 2 for l, h in zip(lo, hi)], [0, 0, 0], color, collide)

    def mirrored_x(x):
        return [2 * mx - x[1], 2 * mx - x[0]]

    for el in field["elements"]:
        copies = [("", False)] + ([("_blue", True)] if el.get("mirror") else [])
        for suffix, flip in copies:
            name = el["name"] + suffix
            color = el.get("mirror_color", el["color"]) if flip else el["color"]
            collide = el.get("collide", True)
            if "box" in el:
                lo, hi = (list(p) for p in el["box"])
                if flip:
                    lo[0], hi[0] = 2 * mx - hi[0], 2 * mx - lo[0]
                add_box(name, lo, hi, color, collide)
            elif "ramp" in el:
                r = el["ramp"]
                x, up = list(r["x"]), r["up"]
                if flip:
                    x = mirrored_x(x)
                    up = {"+x": "-x", "-x": "+x"}.get(up, up)
                add(name, *ramp_box(x, list(r["y"]), r["rise"], up), color, collide)
            else:
                raise SystemExit(f"doctor_urdf: field element {el['name']}: needs box or ramp")

    road = field.get("bumpy_road")
    if road:
        tile, b = road["tile"], road["bump"]
        for suffix, flip in [("", False)] + ([("_blue", True)] if road.get("mirror") else []):
            for row, count in enumerate(road["rows"]):
                y = [road["y0"] + row * tile, road["y0"] + (row + 1) * tile]
                x = [road["x0"], road["x0"] + count * tile]
                if flip:
                    x = mirrored_x(x)
                # The plates are 2.5 mm: drawn, and collided only under the bumps,
                # which sit on them (without the plates they'd stand 6.2 mm proud
                # of the floor instead of 3.7 mm).
                add_box(f"bumpy_road{suffix}_row{row}", [x[0], y[0], 0.0],
                        [x[1], y[1], road["plate_thickness"]], road["color"], bool(road.get("bumps")))
                if not road.get("bumps"):
                    continue
                for i in range(count):
                    tx = [road["x0"] + i * tile, road["x0"] + (i + 1) * tile]
                    if flip:
                        tx = mirrored_x(tx)
                    across_x = (row + i) % 2 == 0  # alternate tiles turn their bumps 90 degrees
                    z = [road["plate_thickness"], road["plate_thickness"] + b["height"]]
                    for k in range(4):
                        o = b["first"] + k * b["pitch"]
                        if across_x:
                            lo = [tx[0] + (tile - b["length"]) / 2, y[0] + o - b["width"] / 2, z[0]]
                            hi = [tx[0] + (tile + b["length"]) / 2, y[0] + o + b["width"] / 2, z[1]]
                        else:
                            lo = [tx[0] + o - b["width"] / 2, y[0] + (tile - b["length"]) / 2, z[0]]
                            hi = [tx[0] + o + b["width"] / 2, y[0] + (tile + b["length"]) / 2, z[1]]
                        add_box(f"bump{suffix}_r{row}_t{i}_{k}", lo, hi, road["bump_color"], True)
        if not road.get("bumps"):
            gap("field: the Bumpy Road's 3.7 mm bumps are off (field yaml bumpy_road.bumps): "
                "the road is flat")
    gap("field: the Visual Markers (Figure 3-10) aren't modelled: their patterns aren't vendored, "
        "and the cameras draw solid colours only")


def fixed_joint(robot, name, parent, child, xyz, rpy):
    j = sub(robot, "joint", name=name, type="fixed")
    sub(j, "parent", link=parent)
    sub(j, "child", link=child)
    sub(j, "origin", xyz=fmt(xyz), rpy=fmt(rpy))


def doctor(spec: dict, spec_dir: str, variant: str, out_dir: str) -> str:
    src = os.path.join(spec_dir, spec["urdf"][variant])
    tree = ET.parse(src)
    robot = tree.getroot()
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}

    def need_link(name, what):
        if name not in links:
            raise SystemExit(f"doctor_urdf: {what}: no link '{name}' in {src}")
        return links[name]

    # Meshes by absolute path: the doctored URDF lives elsewhere.
    for mesh in robot.iter("mesh"):
        fn = mesh.get("filename")
        if fn.startswith("package://"):
            raise SystemExit(f"doctor_urdf: {src} uses package:// meshes; use the standalone URDF")
        mesh.set("filename", os.path.abspath(os.path.join(os.path.dirname(src), fn)))

    # The coarse URDF's visuals are 80 MB of CAD meshes: dropped, so MuJoCo
    # draws the collision shapes instead. The primitives URDF's are boxes
    # and cylinders, and Achilles draws only visuals, so they stay, each
    # named material's colour written into the visual itself (the
    # converter reads only an inline colour). The armor visuals are added
    # below.
    colors = {}
    for m in robot.findall("material"):
        c = m.find("color")
        if m.get("name") and c is not None:
            colors[m.get("name")] = c.get("rgba")
    dropped = 0
    for link in links.values():
        for v in link.findall("visual"):
            g = v.find("geometry")
            if g is None or len(g) == 0 or g[0].tag == "mesh":
                link.remove(v)
                dropped += 1
                continue
            mat = v.find("material")
            if mat is not None and mat.find("color") is None and mat.get("name") in colors:
                sub(mat, "color", rgba=colors[mat.get("name")])
    if dropped:
        gap(f"{variant}: the vendored CAD visual meshes are not rendered; cameras see collision shapes")
    for name, o in spec.get("collision_overrides", {}).items():
        link = need_link(name, f"collision override {name}")
        for c in link.findall("collision"):
            link.remove(c)
        for i, layer in enumerate(o["octagon_layers"]):
            for k, (size, xyz, rpy) in enumerate(octagon_boxes(layer)):
                box_geom(link, "collision", size, xyz=xyz, rpy=rpy, name=f"{name}_fit{i}_{k}")
    # MuJoCo's importer options.
    for old in robot.findall("mujoco"):
        robot.remove(old)
    mj = ET.Element("mujoco")
    sub(mj, "compiler", **spec.get("mujoco_compiler", {}))
    if spec.get("mujoco_option"):
        sub(mj, "option", **spec["mujoco_option"])
    robot.insert(0, mj)

    # Floor and floating base.
    roots = set(links) - {j.find("child").get("link") for j in joints.values()}
    if len(roots) != 1:
        raise SystemExit(f"doctor_urdf: {src} has {len(roots)} root links; expected one")
    root = roots.pop()
    world = spec["world"]
    world_link = sub(robot, "link", name="world")
    with open(os.path.join(spec_dir, world["field"])) as f:
        build_field(world_link, yaml.safe_load(f))
    fj = sub(robot, "joint", name="floating_base", type="floating")
    sub(fj, "parent", link="world")
    sub(fj, "child", link=root)
    sub(fj, "origin", xyz=fmt(world["spawn"]["xyz"]), rpy=fmt(world["spawn"]["rpy"]))

    # Sensor frames.
    for name, f in spec.get("frames", {}).items():
        need_link(f["parent"], f"frame {name}")
        link = sub(robot, "link", name=name)
        inertial(link, 1e-4)
        fixed_joint(robot, f"{name}_mount", f["parent"], name, f["xyz"], f["rpy"])

    # Omni-wheel rollers.
    ow = spec.get("omni_wheels")
    if ow:
        r_roll, length = ow["roller_radius"], ow["roller_length"]
        centre = ow["wheel_radius"] - r_roll
        # Measured in beyblade runs (truth-logged wheel encoders vs the chassis's true motion):
        # a roller touches down still spinning as it was a wheel turn ago, and the soft
        # (MuJoCo-model, both backends) contact spins it up over ~10 ms, longer than it stays
        # down, so it skids sideways; the polygon of contacts also hops the hub.
        gap("omni wheels: discrete rollers on soft contacts skid sideways while the chassis spins and "
            "translates (~0.8 m/s rms at 13 rad/s), so wheel odometry runs ahead by ~0.3 s x the "
            "chassis's acceleration: about 4%/m of drift in beyblade, not a real wheel's")
        for wheel in ow["wheels"]:
            link = need_link(wheel, "omni wheel")
            joint = next((j for j in joints.values() if j.find("child").get("link") == wheel), None)
            if joint is None or joint.find("axis") is None:
                raise SystemExit(f"doctor_urdf: no axis for wheel {wheel}")
            axle = norm([float(v) for v in joint.find("axis").get("xyz").split()])
            for c in link.findall("collision"):
                link.remove(c)
            if variant == "achilles" and ow.get("roller_rgba"):
                # Drawn as its hub, inside the rollers drawn below.
                for cyl in link.iter("cylinder"):
                    cyl.set("radius", f"{ow['wheel_radius'] - 2 * r_roll:.9g}")
            basis = frame_with_z(axle)
            u = [basis[0][0], basis[1][0], basis[2][0]]
            v = [basis[0][1], basis[1][1], basis[2][1]]
            n = ow["rollers_per_row"]
            for row in range(ow["rows"]):
                offset = (row - (ow["rows"] - 1) / 2) * ow["row_spacing"]
                for k in range(n):
                    theta = 2 * math.pi * (k + 0.5 * row) / n
                    radial = [math.cos(theta) * u[i] + math.sin(theta) * v[i] for i in range(3)]
                    tangent = [-math.sin(theta) * u[i] + math.cos(theta) * v[i] for i in range(3)]
                    pos = [centre * radial[i] + offset * axle[i] for i in range(3)]
                    rname = f"{wheel}_roller_{row}_{k}"
                    rl = sub(robot, "link", name=rname)
                    m = ow["roller_mass"]
                    ia = 0.5 * m * r_roll ** 2
                    ir = m * (3 * r_roll ** 2 + length ** 2) / 12
                    inertial(rl, m, (ir, ir, ia))
                    col = sub(rl, "collision", name=f"{rname}_geom")
                    sub(col, "origin", xyz="0 0 0", rpy="0 0 0")
                    if ow.get("roller_shape", "cylinder") == "sphere":
                        sub(sub(col, "geometry"), "sphere", radius=f"{r_roll:.9g}")
                        ia = ir = 0.4 * m * r_roll ** 2
                        rl.find("inertial").find("inertia").attrib.update(
                            ixx=f"{ir:.9g}", iyy=f"{ir:.9g}", izz=f"{ia:.9g}")
                    else:
                        sub(sub(col, "geometry"), "cylinder", radius=f"{r_roll:.9g}", length=f"{length:.9g}")
                    if variant == "achilles" and ow.get("roller_rgba"):
                        vis = sub(rl, "visual", name=f"{rname}_vis")
                        vis.append(copy.deepcopy(col.find("origin")))
                        vis.append(copy.deepcopy(col.find("geometry")))
                        sub(sub(vis, "material", name=f"{rname}_mat"), "color", rgba=fmt(ow["roller_rgba"]))
                    rj = sub(robot, "joint", name=f"{rname}_spin", type="continuous")
                    sub(rj, "parent", link=wheel)
                    sub(rj, "child", link=rname)
                    sub(rj, "origin", xyz=fmt(pos), rpy=fmt(rpy_from_matrix(frame_with_z(tangent))))
                    sub(rj, "axis", xyz="0 0 1")
                    sub(rj, "dynamics", damping=ow.get("roller_friction_damping", 0.0), friction=0)

    # Flywheels and agitator.
    for name, mech in spec.get("mechanisms", {}).items():
        need_link(mech["parent"], f"mechanism {name}")
        link = sub(robot, "link", name=name)
        inertial(link, mech["mass"], mech["inertia"])
        j = sub(robot, "joint", name=f"joint_{name}", type="continuous")
        sub(j, "parent", link=mech["parent"])
        sub(j, "child", link=name)
        sub(j, "origin", xyz=fmt(mech["xyz"]), rpy="0 0 0")
        sub(j, "axis", xyz=fmt(mech["axis"]))

    # Armor plates.
    armor = spec.get("armor")
    if armor:
        team = {"blue": [0.1, 0.35, 1.0, 1.0], "red": [1.0, 0.15, 0.1, 1.0]}[armor["team_color"]]
        p, bar, sticker = armor["plate"], armor["light_bar"], armor["sticker"]
        for name, plate in armor["plates"].items():
            need_link(plate["parent"], f"armor {name}")
            link = sub(robot, "link", name=name)
            inertial(link, 0.05)
            size = [p["thickness"], p["width"], p["height"]]
            box_geom(link, "collision", size, name=f"{name}_plate")
            box_geom(link, "visual", size, rgba=p["rgba"], name=f"{name}_plate")
            box_geom(link, "visual", [0.001, sticker["width"], sticker["height"]],
                     xyz=[p["thickness"] / 2 + 0.0005, 0, 0], rgba=sticker["rgba"], name=f"{name}_sticker")
            for side, sign in (("l", 1), ("r", -1)):
                box_geom(link, "visual", [bar["thickness"], bar["width"], bar["height"]],
                         xyz=[0, sign * bar["offset"], 0], rgba=team, name=f"{name}_bar_{side}")
            fixed_joint(robot, f"{name}_mount", plate["parent"], name, plate["xyz"], plate["rpy"])
        gap(f"{variant}: armor light bars are colored, not emissive (URDF has no emission)")

    # Visual overrides (Achilles draws visuals; MuJoCo draws the collision
    # shapes, and a URDF visual there would be one more colliding geom).
    # Last, so they can name the links added above (sensor frames).
    if variant == "achilles":
        links.update((l.get("name"), l) for l in robot.findall("link"))
        for name, o in spec.get("visual_overrides", {}).items():
            link = need_link(name, f"visual override {name}")
            for v in link.findall("visual"):
                link.remove(v)
            if o.get("from_collision"):
                # The link's collision shapes, drawn: what the engines collide
                # with and the lidars see is what the cameras see.
                for i, c in enumerate(link.findall("collision")):
                    v = sub(link, "visual", name=f"{name}_vis{i}")
                    v.append(copy.deepcopy(c.find("origin")))
                    v.append(copy.deepcopy(c.find("geometry")))
                    sub(sub(v, "material", name=f"{name}_vis{i}_mat"), "color", rgba=fmt(o["rgba"]))
            for i, s in enumerate(o.get("shapes", [o] if "rgba" in o and not o.get("from_collision") else [])):
                v = sub(link, "visual", name=f"{name}_shape{i}")
                sub(v, "origin", xyz=fmt(s.get("xyz", [0, 0, 0])), rpy=fmt(s.get("rpy", [0, 0, 0])))
                geom = sub(v, "geometry")
                if "cylinder" in s:
                    sub(geom, "cylinder", radius=f"{s['cylinder']['radius']:.9g}", length=f"{s['cylinder']['length']:.9g}")
                elif "sphere" in s:
                    sub(geom, "sphere", radius=f"{s['sphere']['radius']:.9g}")
                else:
                    sub(geom, "box", size=fmt(s["box"]))
                sub(sub(v, "material", name=f"{name}_shape{i}_mat"), "color", rgba=fmt(s["rgba"]))

    ET.indent(tree, space="  ")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"sentry_{variant}.urdf")
    with open(out, "w") as f:
        f.write(f"<?xml version=\"1.0\"?>\n<!-- Generated by tools/doctor_urdf.py from {os.path.relpath(src, out_dir)}"
                f" and urdf_doctor.yaml. Don't edit: edit urdf_doctor.yaml. -->\n")
        tree.write(f, encoding="unicode")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("spec")
    ap.add_argument("out_dir")
    args = ap.parse_args()
    with open(args.spec) as f:
        spec = yaml.safe_load(f)
    for variant in ("mujoco", "achilles"):
        print(doctor(spec, os.path.dirname(os.path.abspath(args.spec)), variant, args.out_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
