#!/usr/bin/env python3
"""
Stands in for the Odin's ROS 2 driver (odin_ros_driver host_sdk_sample) in a
sim run, for the 2026-27 autonav stack (project-otto feature/autonav_framework,
autonav/): scripts/run.sh --autonav starts it in the ROS 2 container.

It publishes what the stack reads from the driver:

  /odin1/cloud_raw  sensor_msgs/PointCloud2 in the `imu` frame, one per scan:
                    the hub's fans of beams at the Odin's mount (the model's
                    `odin` frame), 126 degrees wide and tilted to sample the
                    band the stack's maps are drawn for, each read off its
                    LD19-protocol PTY (<runtime dir>/odin_f<k>) and merged.
  TF odom -> imu    the Odin's pose, at 10 Hz: its true motion since start,
                    from the hub's truth log (<runtime dir>/truth.jsonl), with
                    the error the Odin was measured to have (assets/sentry/
                    odin.yaml): one registration offset per relocalization,
                    correlated heavy-tailed noise, nothing until its first fix
                    ~10 s after start, and sometimes a wrong fix.

and, since the hub's clock is the sim's, /clock: every node of the stack runs
with use_sim_time, on sim time.

    hwemu_odin.py RUNTIME_DIR ODIN_YAML

HWEMU_ODIN_SEED (default 1) seeds the error; HWEMU_ODIN_WRONG_FIX=1 forces a
wrong first fix, 0 rules it out; HWEMU_ODIN_FIRST_FIX_S overrides when the
first fix comes (as if the driver had started that long before the sim). Every draw and fix goes to
<runtime dir>/autonav/odin_events.jsonl, and the Odin's output (beside truth)
to <runtime dir>/autonav/odin_pose.jsonl.
"""

import json
import math
import os
import random
import struct
import sys

import yaml

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "hw-emulator", "python"))
from hwemu import gaps  # noqa: E402

# The model's chain to the Odin (assets/sentry/urdf_doctor.yaml): base_footprint -> base_link
# (0.062679 up) -> turret_major (0.0776 up, turning with joint_major) -> odin.
MAJOR_HEIGHT = 0.062679 + 0.0776
ODIN_MOUNT = (-0.155631, -0.034668, 0.1635, math.pi)  # x, y, z, yaw in turret_major

PACKET = 47


def stamp(t: float) -> TimeMsg:
    sec = int(math.floor(t))
    return TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)


def odin_pose(row):
    """The Odin's planar world pose (x, y, z, yaw) from a truth row."""
    yaw_major = row["yaw"] + row["encoders"]["major_yaw_encoder"]
    mx, my, mz, myaw = ODIN_MOUNT
    c, s = math.cos(yaw_major), math.sin(yaw_major)
    return (row["x"] + c * mx - s * my, row["y"] + s * mx + c * my, row["z"] + MAJOR_HEIGHT + mz, yaw_major + myaw)


class OdinError:
    """The Odin's pose error (assets/sentry/odin.yaml): eps = R(heading)^T b_k + eta, in body-aligned axes."""

    AXES = ("along", "cross", "vertical", "roll", "pitch", "yaw")

    def __init__(self, cfg, seed, force_wrong, log):
        self._cfg, self._rng, self._log = cfg, random.Random(seed), log
        self._dt = 1.0 / cfg["rate_hz"]
        self._eta = dict.fromkeys(self.AXES, 0.0)
        self._b = None             # map frame: x, y, z (m), roll, pitch, yaw (rad)
        self._wrong_until = None
        self._force_wrong = force_wrong
        self.fixed = False

    def _student(self, nu):
        """Unit-variance Student-t (nu None: Gaussian)."""
        g = self._rng.gauss(0.0, 1.0)
        if nu is None:
            return g
        chi2 = self._rng.gammavariate(nu / 2.0, 2.0)
        return g / math.sqrt(chi2 / nu) * math.sqrt((nu - 2.0) / nu)

    def _relocalize(self, t, why):
        r = self._cfg["registration"]
        g = self._rng.gauss
        self._b = [g(0, r["x"]), g(0, r["y"]), g(0, r["z"]),
                   math.radians(g(0, r["roll"])), math.radians(g(0, r["pitch"])), math.radians(g(0, r["yaw"]))]
        self.fixed = True
        self._log({"t": round(t, 3), "event": "relocalized", "why": why,
                   "b": {"x": self._b[0], "y": self._b[1], "z": self._b[2],
                         "roll_deg": math.degrees(self._b[3]), "pitch_deg": math.degrees(self._b[4]),
                         "yaw_deg": math.degrees(self._b[5])}})

    def first_fix(self, t):
        self._relocalize(t, "first fix")
        w = self._cfg["wrong_fix"]
        wrong = self._rng.random() < w["probability"] if self._force_wrong is None else self._force_wrong
        if wrong:
            o = w["offset"]
            heading = self._rng.uniform(-math.pi, math.pi)
            sign = lambda: self._rng.choice((-1.0, 1.0))  # noqa: E731
            self._b[0] += o["horizontal"] * math.cos(heading)
            self._b[1] += o["horizontal"] * math.sin(heading)
            self._b[2] += o["z"]
            self._b[3] += sign() * math.radians(o["roll"])
            self._b[4] += sign() * math.radians(o["pitch"])
            self._b[5] += sign() * math.radians(o["yaw"])
            self._wrong_until = t + w["lasts_s"]
            self._log({"t": round(t, 3), "event": "wrong fix", "until": round(self._wrong_until, 3),
                       "b": {"x": self._b[0], "y": self._b[1], "z": self._b[2],
                             "roll_deg": math.degrees(self._b[3]), "pitch_deg": math.degrees(self._b[4]),
                             "yaw_deg": math.degrees(self._b[5])}})

    def step(self, t, speed):
        """Advance eta by one output period, at this speed (m/s)."""
        if self._wrong_until is not None and t >= self._wrong_until:
            self._wrong_until = None
            self._relocalize(t, "corrected a wrong fix")
        params = self._cfg["moving"] if speed > self._cfg["still_below"] else self._cfg["still"]
        for axis in self.AXES:
            p = params[axis]
            a = math.exp(-self._dt / p["tau"])
            sigma = p["sigma"] if axis in ("along", "cross", "vertical") else math.radians(p["sigma"])
            self._eta[axis] = a * self._eta[axis] + sigma * math.sqrt(1.0 - a * a) * self._student(p.get("nu"))

    def apply(self, x, y, z, roll, pitch, yaw, travel):
        """The output pose: truth (+) eps, eta's along/cross axes along `travel` (rad)."""
        e = self._eta
        c, s = math.cos(travel), math.sin(travel)
        bx, by, bz, br, bp, byaw = self._b
        return (x + bx + c * e["along"] - s * e["cross"], y + by + s * e["along"] + c * e["cross"],
                z + bz + e["vertical"], roll + br + e["roll"], pitch + bp + e["pitch"], yaw + byaw + e["yaw"])


def quat_rpy(roll, pitch, yaw):
    """(x, y, z, w) of fixed-axis roll, pitch, yaw."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class TruthTail:
    """New rows of the hub's truth log, as they're flushed."""

    def __init__(self, path):
        self._path, self._f, self._buf = path, None, b""

    def rows(self):
        if self._f is None:
            if not os.path.exists(self._path):
                return []
            self._f = open(self._path, "rb")
        self._buf += self._f.read()
        *lines, self._buf = self._buf.split(b"\n")
        out = []
        for line in lines:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
        return out


class Ld19Reader:
    """Packets from an LD19-protocol byte stream: (sent ms mod 30000, [(ccw angle rad, range m or None)] x 12)."""

    def __init__(self, path):
        self._fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        import termios
        import tty
        tty.setraw(self._fd)
        termios.tcflush(self._fd, termios.TCIFLUSH)
        self._buf = b""

    def packets(self):
        try:
            self._buf += os.read(self._fd, 65536)
        except BlockingIOError:
            pass
        out = []
        while True:
            i = self._buf.find(b"\x54\x2c")
            if i < 0 or len(self._buf) - i < PACKET:
                self._buf = self._buf[i:] if i >= 0 else self._buf[-1:]
                break
            p = self._buf[i:i + PACKET]
            self._buf = self._buf[i + PACKET:]
            ms = struct.unpack_from("<H", p, 44)[0]
            start = struct.unpack_from("<H", p, 4)[0] / 100.0
            end = struct.unpack_from("<H", p, 42)[0] / 100.0
            span = (end - start) % 360.0
            pts = []
            for k in range(12):
                d, _ = struct.unpack_from("<HB", p, 6 + 3 * k)
                cw = start + span * k / 11.0
                pts.append((math.radians(-cw), d / 1000.0 if d > 0 else None))
            out.append((ms, pts))
        return out


class OdinSim(Node):
    def __init__(self, runtime, cfg):
        super().__init__("odin_sim")
        self._cfg = cfg
        self._truth = TruthTail(os.path.join(runtime, "truth.jsonl"))
        # One PTY per fan, each fan tilted down by its pitch about the Odin's y axis.
        self._fans = [(os.path.join(runtime, f"odin_f{k}"), math.radians(p))
                      for k, p in enumerate(cfg["fan_pitch_deg"])]
        self._readers = [None] * len(self._fans)
        self._per_scan = len(self._fans) * cfg["samples"] // 12  # packets in one merged scan
        self._pending = {}   # ms -> [packets, points]
        self._clock_pub = self.create_publisher(Clock, "/clock", QoSProfile(depth=10))
        self._cloud = self.create_publisher(PointCloud2, "/odin1/cloud_raw", qos_profile_sensor_data)
        self._tf = TransformBroadcaster(self)
        self._origin = None
        self._now = None
        self._clouds = 0
        self.create_timer(0.002, self._poll)
        # The stack's answer, kept for scripts/autonav_check.py-style checks against truth:
        # <runtime dir>/autonav/localization.jsonl, one {t, x, y, yaw} per /localization/pose.
        os.makedirs(os.path.join(runtime, "autonav"), exist_ok=True)
        self._loc_log = open(os.path.join(runtime, "autonav", "localization.jsonl"), "w", buffering=1)
        self._pose_log = open(os.path.join(runtime, "autonav", "odin_pose.jsonl"), "w", buffering=1)
        events = open(os.path.join(runtime, "autonav", "odin_events.jsonl"), "w", buffering=1)
        seed = int(os.environ.get("HWEMU_ODIN_SEED", "1"))
        force = {"1": True, "0": False}.get(os.environ.get("HWEMU_ODIN_WRONG_FIX", ""))

        def log(ev):
            events.write(json.dumps(ev) + "\n")
            self.get_logger().info(f"odin: {ev}")
        log({"t": 0.0, "event": "start", "seed": seed, "wrong_fix": force})
        if os.environ.get("HWEMU_ODIN_FIRST_FIX_S"):
            cfg["first_fix_s"] = float(os.environ["HWEMU_ODIN_FIRST_FIX_S"])
        log({"t": 0.0, "event": "first fix due", "t_fix": cfg["first_fix_s"]})
        self._error = OdinError(cfg, seed, force, log)
        self._period = 1.0 / cfg["rate_hz"]
        self._next_out = None
        self._last_out = None   # (t, x, y) of the last output's truth, for speed and travel direction
        self.create_subscription(Odometry, "/localization/pose", self._on_localization, 10)
        gaps.report("odin", "fans",
                    f"the Odin's cloud is {len(self._fans)} flat fans of {cfg['samples']} beams across its "
                    f"{cfg['fov_deg']:g}-degree field of view, tilted to sample the 0.10-0.40 m band (not its "
                    "dense 3-D pattern), and it doesn't see the robot itself (its CAD mount is inside the "
                    "model's coarse turret major)")
        gaps.report("odin", "pose",
                    "the Odin's pose is truth plus its measured error (registration offset per relocalization, "
                    "heavy-tailed correlated noise, first fix ~10 s after start, sometimes a wrong fix: "
                    "assets/sentry/odin.yaml), planar truth only (no roll or pitch of the robot itself), and "
                    "never stalls: nothing covers its window in the sim")

    def _poll(self):
        for row in self._truth.rows():
            if "encoders" not in row or "major_yaw_encoder" not in row["encoders"]:
                continue
            self._now = row["t"]
            self._clock_pub.publish(Clock(clock=stamp(self._now)))
            x, y, z, yaw = odin_pose(row)
            if self._origin is None:
                self._origin = (x, y, z, yaw)
                self._next_out = self._now
            if self._now + 1e-6 < self._next_out:
                continue
            self._next_out += self._period
            self._output(x, y, z, yaw)
        for k, (port, _) in enumerate(self._fans):
            if self._readers[k] is None:
                if not os.path.exists(port):
                    continue
                self._readers[k] = Ld19Reader(port)
            for ms, pts in self._readers[k].packets():
                group = self._pending.setdefault(ms, [0, []])
                group[0] += 1
                pitch = self._fans[k][1]
                c, s = math.cos(pitch), math.sin(pitch)
                for a, r in pts:
                    if r is not None:
                        px, py = r * math.cos(a), r * math.sin(a)
                        group[1].append((c * px, py, -s * px))
        for ms in sorted(self._pending):
            n, pts = self._pending[ms]
            if n < self._per_scan and len(self._pending) < 4:
                break
            del self._pending[ms]
            if self._now is None:
                continue
            # Stamped when the hub cast it (its packets' time, sim ms mod 30000), not when it's
            # read here: AMCL places the scan with the Odin's pose at that stamp.
            age = ((round(self._now * 1000) - ms) % 30000) / 1000.0
            if age > 15.0:
                age -= 30.0
            self._publish_cloud(pts, self._now - age)

    def _output(self, x, y, z, yaw):
        """One of the Odin's 10 Hz poses, with its error, on TF odom -> imu."""
        t = self._now
        speed, travel = 0.0, yaw
        if self._last_out is not None and t > self._last_out[0]:
            dx, dy = x - self._last_out[1], y - self._last_out[2]
            speed = math.hypot(dx, dy) / (t - self._last_out[0])
            if speed > self._cfg["still_below"]:
                travel = math.atan2(dy, dx)
        self._last_out = (t, x, y)
        if not self._error.fixed:
            if t < self._cfg["first_fix_s"]:
                return  # not localized yet: the driver publishes no pose
            self._error.first_fix(t)
        self._error.step(t, speed)
        ex, ey, ez, er, ep, eyaw = self._error.apply(x, y, z, 0.0, 0.0, yaw, travel)
        self._pose_log.write(json.dumps({"t": round(t, 4), "x": ex, "y": ey, "z": ez, "yaw": eyaw,
                                         "true": [x, y, z, yaw]}) + "\n")
        ox, oy, oz, oyaw = self._origin
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        tf = TransformStamped()
        tf.header.stamp = stamp(t)
        tf.header.frame_id = "odom"
        tf.child_frame_id = "imu"
        tf.transform.translation.x = c * (ex - ox) - s * (ey - oy)
        tf.transform.translation.y = s * (ex - ox) + c * (ey - oy)
        tf.transform.translation.z = ez - oz
        q = quat_rpy(er, ep, eyaw - oyaw)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self._tf.sendTransform(tf)

    def _on_localization(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._loc_log.write(json.dumps({"t": round(t, 4), "x": p.x, "y": p.y, "yaw": yaw}) + "\n")

    def _publish_cloud(self, pts, t):
        msg = PointCloud2()
        msg.header.stamp = stamp(t)
        msg.header.frame_id = "imu"
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(pts), True
        msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
        self._cloud.publish(msg)
        self._clouds += 1
        if self._clouds in (1, 10) or self._clouds % 100 == 0:
            self.get_logger().info(f"cloud {self._clouds}: {len(pts)} points, cast at t={t:.3f} (now {self._now:.3f})")


def main():
    rclpy.init()
    node = OdinSim(sys.argv[1], yaml.safe_load(open(sys.argv[2])))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
