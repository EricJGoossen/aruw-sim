#!/usr/bin/env python3
"""Checks the sim's sentry model against the robot code, and fails loudly.

    tools/check_model.py [--accept-mismatch NAME ...]

The sim never reads aruw-mcb or project-otto; it has its own numbers in
assets/sentry/urdf_doctor.yaml and the vendored URDF. This script is the one
place those sources are read. It compares every number the two sides share,
and exits 1 on any that differ beyond a small tolerance, naming both numbers
and both files. Checked:
- the IMU mounts;
- the camera and lidar poses;
- the wheel radius and wheelbase;
- each motor's and encoder's mounting direction (the firmware's "inverted"
  flags);
- the turret encoders' calibrated zero counts;
- the turret joints' travel against the firmware's angle limits.

When the sim's numbers are deliberately different (real measurements in the
YAML), `--accept-mismatch NAME` (or HWEMU_ACCEPT_MISMATCH="NAME,NAME") lets
that one through. It is still printed.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MCB = os.path.join(ROOT, "aruw-mcb", "aruw-mcb-project", "src", "aruwsrc", "robot", "sentry")
OTTO_CONFIG = os.path.join(ROOT, "project-otto", "sentry_name_config.yaml")
DOCTOR = os.path.join(ROOT, "assets", "sentry", "urdf_doctor.yaml")

TOL_M = 0.0005   # 0.5 mm
TOL_RAD = 0.002  # ~0.1 degree


def cpp_number(text: str) -> float:
    text = re.sub(r"(\d\.?)[fF]\b", r"\1", text.strip())
    text = text.replace("M_PI_2", str(math.pi / 2)).replace("M_PI", str(math.pi))
    if not re.fullmatch(r"[-+*/.()\d\seE]+", text):
        raise ValueError(f"not a numeric literal: {text!r}")
    return float(eval(text, {"__builtins__": {}}))  # noqa: S307 -- digits and operators only


def transform_args(source: str, name: str) -> list[float]:
    """The six numbers of `Transform NAME(x, y, z, rx, ry, rz);` in C++ source."""
    m = re.search(name + r"\s*\(([^;]*?)\)\s*;", source, re.S)
    if not m:
        raise KeyError(name)
    return [cpp_number(v) for v in m.group(1).split(",")]


def transform_array(source: str, name: str) -> list[list[float]]:
    m = re.search(name + r"\s*=\s*\{(.*?)\}\s*;", source, re.S)
    if not m:
        raise KeyError(name)
    return [[cpp_number(v) for v in t.split(",")] for t in re.findall(r"Transform\s*\(([^)]*)\)", m.group(1))]


def constant(source: str, name: str) -> str:
    m = re.search(r"constexpr\s+float\s+" + name + r"\s*=\s*([^;]+);", source)
    if not m:
        raise KeyError(name)
    return m.group(1)


class Checker:
    def __init__(self, accepted: set[str]):
        self.accepted = accepted
        self.failures = 0
        self.checked = 0

    def same(self, name: str, ours, theirs, tol: float, ours_where: str, theirs_where: str) -> None:
        self.checked += 1
        ours = [ours] if isinstance(ours, (int, float)) else list(ours)
        theirs = [theirs] if isinstance(theirs, (int, float)) else list(theirs)
        worst = max(abs(a - b) for a, b in zip(ours, theirs))
        if len(ours) == len(theirs) and worst <= tol:
            return
        tag = "ACCEPTED MISMATCH" if name in self.accepted else "MISMATCH"
        print(f"[check_model] {tag} {name}:\n"
              f"    sim   {fmt(ours)}  ({ours_where})\n"
              f"    robot {fmt(theirs)}  ({theirs_where})", file=sys.stderr)
        if name not in self.accepted:
            self.failures += 1


def fmt(values) -> str:
    return "[" + ", ".join(f"{v:.6g}" for v in values) + "]"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--accept-mismatch", action="append", default=[])
    args = ap.parse_args()
    accepted = set(args.accept_mismatch)
    accepted |= {a for a in os.environ.get("HWEMU_ACCEPT_MISMATCH", "").split(",") if a}

    with open(DOCTOR) as f:
        doctor = yaml.safe_load(f)
    frames = doctor["frames"]
    chassis_h = open(os.path.join(MCB, "sentry_chassis_constants.hpp")).read()
    turret_h = open(os.path.join(MCB, "sentry_turret_constants.hpp")).read()
    drivers_h = open(os.path.join(MCB, "sentry_drivers.hpp")).read()
    with open(OTTO_CONFIG) as f:
        otto = yaml.safe_load(f)
    c = Checker(accepted)
    yml = "assets/sentry/urdf_doctor.yaml"

    def frame_check(frame: str, values, where: str, scale: float = 1.0) -> None:
        f = frames[frame]
        c.same(f"{frame}.xyz", f["xyz"], [v * scale for v in values[:3]], TOL_M, f"{yml} frames.{frame}", where)
        c.same(f"{frame}.rpy", f["rpy"], values[3:6], TOL_RAD, f"{yml} frames.{frame}", where)

    # IMU mounts (MCB).
    for frame, const, header, src in [
        ("chassis_mcb_bmi088", "CHASSIS_MCB_BMI088_MOUNTING_TRANSFORM", chassis_h, "sentry_chassis_constants.hpp"),
        ("chassis_mcb_ism330", "CHASSIS_MCB_ISM330_MOUNTING_TRANSFORM", chassis_h, "sentry_chassis_constants.hpp"),
        ("turret_mcb1_bmi088", "TURRET_MCB1_BMI088_MOUNTING_TRANSFORM", turret_h, "sentry_turret_constants.hpp"),
        ("turret_mcb1_ism330", "TURRET_MCB1_ISM330_MOUNTING_TRANSFORM", turret_h, "sentry_turret_constants.hpp"),
    ]:
        frame_check(frame, transform_args(header, const), f"aruw-mcb {src} {const}")
    fused = transform_array(drivers_h, "turretMajorImuTransforms")
    for i, frame in enumerate(["turret_major_ism330_primary", "turret_major_ism330_secondary", "turret_major_mpu6500"]):
        frame_check(frame, fused[i], f"aruw-mcb sentry_drivers.hpp turretMajorImuTransforms[{i}] (mm)", scale=0.001)

    # Camera and lidar poses (otto).
    cam = otto["turret_geometry"][0]["turret_reference_to_camera_transform"]
    frame_check("turret_camera", cam["translation"] + cam["rotation"],
                "project-otto sentry_name_config.yaml turret_geometry.0.turret_reference_to_camera_transform")
    ard = otto["arducam_cameras"][0]["base_reference_to_camera_transform"]
    frame_check("arducam", ard["translation"] + ard["rotation"],
                "project-otto sentry_name_config.yaml arducam_cameras[0].base_reference_to_camera_transform")
    for i, frame in enumerate(["lidar_front", "lidar_back"]):
        lidar = otto["autonav"]["lidars"][i]
        frame_check(frame, lidar["lidar_position_to_chassis_frame"] + lidar["lidar_orientation_to_chassis_frame"],
                    f"project-otto sentry_name_config.yaml autonav.lidars[{i}]")

    # Wheels: the URDF's geometry against the MCB's kinematics.
    radius = cpp_number(constant(chassis_h, "WHEEL_RADIUS").replace("FUDGE_FACTOR",
                                                                    constant(chassis_h, "FUDGE_FACTOR")))
    wheelbase = cpp_number(constant(chassis_h, "WHEELBASE_RADIUS"))
    c.same("wheel_radius", doctor["omni_wheels"]["wheel_radius"], radius, TOL_M,
           f"{yml} omni_wheels.wheel_radius", "aruw-mcb sentry_chassis_constants.hpp WHEEL_RADIUS")
    urdf = os.path.join(ROOT, "assets", "sentry", doctor["urdf"]["mujoco"])
    tree = ET.parse(urdf).getroot()
    for joint in tree.findall("joint"):
        if joint.get("name", "").endswith("_wheel"):
            x, y, _ = (float(v) for v in joint.find("origin").get("xyz").split())
            c.same(f"wheelbase_radius.{joint.get('name')}", math.hypot(x, y), wheelbase, TOL_M,
                   f"{os.path.relpath(urdf, ROOT)} {joint.get('name')} origin",
                   "aruw-mcb sentry_chassis_constants.hpp WHEELBASE_RADIUS")

    # Motors and encoders: direction (the firmware's inverted flags) and
    # calibrated zeros, device by device. Each entry: the devices.yaml device,
    # the firmware object and which constructor argument is its inverted flag.
    control = open(os.path.join(MCB, "sentry_control.cpp")).read()
    with open(os.path.join(ROOT, "assets", "sentry", "devices.yaml")) as f:
        devices = {d["name"]: d for d in yaml.safe_load(f)["devices"]}
    ctor = {}
    for m in re.finditer(r"(?:DjiMotor|LampreyEncoder|CanEncoder)\s+(\w+)\s*\(|\.(\w+)\s*=\s*tap::motor::DjiMotor\s*\(",
                         control):
        name = m.group(1) or m.group(2)
        depth, i = 1, m.end()
        while depth and i < len(control):
            depth += {"(": 1, ")": -1}.get(control[i], 0)
            i += 1
        args, depth, cur = [], 0, ""
        for ch in control[m.end():i - 1]:
            if ch == "," and depth == 0:
                args.append(cur.strip())
                cur = ""
                continue
            depth += {"(": 1, ")": -1, "{": 1, "}": -1}.get(ch, 0)
            cur += ch
        args.append(cur.strip())
        ctor[name] = args
    constants = turret_h + chassis_h

    def flag(text: str) -> bool:
        text = text.strip()
        if text in ("true", "false"):
            return text == "true"
        m = re.search(r"constexpr\s+bool\s+" + re.escape(text.split("::")[-1]) + r"\s*=\s*(true|false)", constants)
        if not m:
            raise KeyError(f"can't resolve inverted flag {text!r}")
        return m.group(1) == "true"

    for device, (obj, arg) in {
        "rf_wheel_motor": ("rightFrontMotor", 3), "lf_wheel_motor": ("leftFrontMotor", 3),
        "lb_wheel_motor": ("leftBackMotor", 3), "rb_wheel_motor": ("rightBackMotor", 3),
        "major_yaw_motor": ("turretMajorYawMotor", 3), "minor_yaw_motor": ("yawMotor", 3),
        "minor_pitch_motor": ("pitchMotor", 3), "flywheel_left_motor": ("turretWidowFrictionWheelLeft", 3),
        "flywheel_right_motor": ("turretWidowFrictionWheelRight", 3),
        "major_yaw_lamprey": ("turretMajorYawLamprey", 4), "major_yaw_can_encoder": ("turretMajorYawCanEncoder", 3),
    }.items():
        inverted = flag(ctor[obj][arg])
        ours = (devices[device].get("params") or {}).get("direction", 1)
        c.same(f"{device}.direction", ours, -1 if inverted else 1, 0.0,
               f"assets/sentry/devices.yaml {device} params.direction",
               f"aruw-mcb sentry_control.cpp {obj} inverted={str(inverted).lower()}")

    def config_block(namespace: str, name: str) -> str:
        ns = turret_h.index("namespace " + namespace)
        start = turret_h.index(name + " = {", ns)
        return turret_h[start:turret_h.index("};", start)]

    for device, ns, cfg in [("major_yaw_motor", "turretMajor", "YAW_MOTOR_CONFIG"),
                            ("minor_yaw_motor", "turretWidow", "YAW_MOTOR_CONFIG"),
                            ("minor_pitch_motor", "turretWidow", "PITCH_MOTOR_CONFIG")]:
        block = config_block(ns, cfg)
        start = float(re.search(r"\.startEncoderValue\s*=\s*(\d+)", block).group(1))
        ours = (devices[device].get("params") or {}).get("encoder_offset", 0)
        c.same(f"{device}.encoder_offset", ours, start, 0.5, f"assets/sentry/devices.yaml {device} params.encoder_offset",
               f"aruw-mcb sentry_turret_constants.hpp {ns}::{cfg}.startEncoderValue")

    # Joint travel: the model must allow at least what the firmware commands.
    def angle(text: str) -> float:
        text = text.strip()
        text = re.sub(r"modm::toRadian\(([^)]*)\)", lambda m: f"({m.group(1)})*{math.pi}/180", text)
        for name in re.findall(r"[A-Z_]{3,}", text):
            m = re.search(r"constexpr\s+float\s+" + name + r"\s*=\s*([^;]+);", turret_h)
            text = text.replace(name, f"({angle(m.group(1))})" if m else name)
        return cpp_number(text.replace(".f", ".0").replace("f ", " "))

    joints = {j.get("name"): j for j in tree.findall("joint")}
    for joint, cfg in [("joint_minor_yaw", "YAW_MOTOR_CONFIG"), ("joint_minor_pitch", "PITCH_MOTOR_CONFIG")]:
        block = config_block("turretWidow", cfg)
        lo = angle(re.search(r"\.minAngle\s*=\s*([^,]+),", block).group(1))
        hi = angle(re.search(r"\.maxAngle\s*=\s*([^,]+),", block).group(1))
        limit = joints[joint].find("limit")
        ours_lo, ours_hi = float(limit.get("lower")), float(limit.get("upper"))
        c.checked += 1
        if ours_lo > lo + TOL_RAD or ours_hi < hi - TOL_RAD:
            name = f"{joint}.limits"
            tag = "ACCEPTED MISMATCH" if name in accepted else "MISMATCH"
            print(f"[check_model] {tag} {name}: the model allows less travel than the firmware commands:\n"
                  f"    sim   [{ours_lo:.4g}, {ours_hi:.4g}] rad  ({os.path.relpath(urdf, ROOT)} {joint} limit)\n"
                  f"    robot [{lo:.4g}, {hi:.4g}] rad  (aruw-mcb sentry_turret_constants.hpp turretWidow::{cfg})",
                  file=sys.stderr)
            if name not in accepted:
                c.failures += 1

    print(f"[check_model] {c.checked} values checked, {c.failures} mismatch(es)"
          + ("" if not accepted else f", accepted: {', '.join(sorted(accepted))}"), file=sys.stderr)
    return 1 if c.failures else 0


if __name__ == "__main__":
    sys.exit(main())
