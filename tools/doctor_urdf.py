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

    # The vendored visuals are 80 MB of CAD meshes: the sim renders the
    # collision shapes instead, plus the armor visuals added below.
    dropped = 0
    for link in links.values():
        for v in link.findall("visual"):
            link.remove(v)
            dropped += 1
    if dropped:
        gap(f"{variant}: the vendored CAD visual meshes are not rendered; cameras see collision shapes")

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
    floor = sub(robot, "link", name="world")
    fx, fy = world["floor"]["size"]
    t = world["floor"]["thickness"]
    box_geom(floor, "collision", [fx, fy, t], xyz=[fx / 2 - 1.0, fy / 2 - 1.0, -t / 2], name="floor")
    box_geom(floor, "visual", [fx, fy, t], xyz=[fx / 2 - 1.0, fy / 2 - 1.0, -t / 2],
             rgba=[0.55, 0.55, 0.5, 1], name="floor")
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
        for wheel in ow["wheels"]:
            link = need_link(wheel, "omni wheel")
            joint = next((j for j in joints.values() if j.find("child").get("link") == wheel), None)
            if joint is None or joint.find("axis") is None:
                raise SystemExit(f"doctor_urdf: no axis for wheel {wheel}")
            axle = norm([float(v) for v in joint.find("axis").get("xyz").split()])
            for c in link.findall("collision"):
                link.remove(c)
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
