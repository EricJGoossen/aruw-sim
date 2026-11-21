#!/usr/bin/env python3
"""
Stands in for the Odin's ROS 2 driver (odin_ros_driver host_sdk_sample) in a
sim run, for the 2026-27 autonav stack (project-otto feature/autonav_framework,
autonav/): scripts/run.sh --autonav starts it in the ROS 2 container.

It publishes what the stack reads from the driver:

  /odin1/cloud_raw  sensor_msgs/PointCloud2 in the `imu` frame, one per scan:
                    the hub's 360-degree lidar at the Odin's mount (the model's
                    `odin` frame), read off its LD19-protocol PTY
                    (<runtime dir>/odin). One horizontal ring, not the Odin's
                    3-D cloud: the localization node cuts its 2-D scan from a
                    band around this height anyway.
  TF odom -> imu    the Odin's own odometry: here its true motion since start,
                    from the hub's truth log (<runtime dir>/truth.jsonl).

and, since the hub's clock is the sim's, /clock: every node of the stack runs
with use_sim_time, on sim time.

    hwemu_odin.py RUNTIME_DIR
"""

import json
import math
import os
import struct
import sys
import time

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
    """Scans from an LD19-protocol byte stream: lists of (ccw angle rad, range m or None)."""

    def __init__(self, path):
        self._fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        import termios
        import tty
        tty.setraw(self._fd)
        termios.tcflush(self._fd, termios.TCIFLUSH)
        self._buf = b""
        self._scan = []
        self._last_angle = None

    def scans(self):
        try:
            self._buf += os.read(self._fd, 65536)
        except BlockingIOError:
            pass
        done = []
        while True:
            i = self._buf.find(b"\x54\x2c")
            if i < 0 or len(self._buf) - i < PACKET:
                self._buf = self._buf[i:] if i >= 0 else self._buf[-1:]
                break
            p = self._buf[i:i + PACKET]
            self._buf = self._buf[i + PACKET:]
            start = struct.unpack_from("<H", p, 4)[0] / 100.0
            end = struct.unpack_from("<H", p, 42)[0] / 100.0
            span = (end - start) % 360.0
            for k in range(12):
                d, _ = struct.unpack_from("<HB", p, 6 + 3 * k)
                cw = start + span * k / 11.0
                if self._last_angle is not None and cw < self._last_angle - 180.0:
                    done.append(self._scan)  # wrapped: one revolution
                    self._scan = []
                self._last_angle = cw
                self._scan.append((math.radians(-cw), d / 1000.0 if d > 0 else None))
        return done


class OdinSim(Node):
    def __init__(self, runtime):
        super().__init__("odin_sim")
        self._truth = TruthTail(os.path.join(runtime, "truth.jsonl"))
        self._port = os.path.join(runtime, "odin")
        self._reader = None
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
        self.create_subscription(Odometry, "/localization/pose", self._on_localization, 10)
        gaps.report("odin", "planar",
                    "the Odin is a 360-degree horizontal lidar ring at its mount (one plane at 0.30 m, "
                    "not its 3-D cloud); its odometry (TF odom -> imu) is the sim's truth, with no drift")

    def _poll(self):
        for row in self._truth.rows():
            if "encoders" not in row or "major_yaw_encoder" not in row["encoders"]:
                continue
            self._now = row["t"]
            self._clock_pub.publish(Clock(clock=stamp(self._now)))
            x, y, z, yaw = odin_pose(row)
            if self._origin is None:
                self._origin = (x, y, z, yaw)
            ox, oy, oz, oyaw = self._origin
            c, s = math.cos(-oyaw), math.sin(-oyaw)
            tf = TransformStamped()
            tf.header.stamp = stamp(self._now)
            tf.header.frame_id = "odom"
            tf.child_frame_id = "imu"
            tf.transform.translation.x = c * (x - ox) - s * (y - oy)
            tf.transform.translation.y = s * (x - ox) + c * (y - oy)
            tf.transform.translation.z = z - oz
            d = yaw - oyaw
            tf.transform.rotation.z = math.sin(d / 2)
            tf.transform.rotation.w = math.cos(d / 2)
            self._tf.sendTransform(tf)
        if self._reader is None:
            if not os.path.exists(self._port):
                return
            self._reader = Ld19Reader(self._port)
        for scan in self._reader.scans():
            if self._now is None:
                continue
            self._publish_cloud(scan)

    def _on_localization(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._loc_log.write(json.dumps({"t": round(t, 4), "x": p.x, "y": p.y, "yaw": yaw}) + "\n")

    def _publish_cloud(self, scan):
        pts = [(r * math.cos(a), r * math.sin(a), 0.0) for a, r in scan if r is not None]
        msg = PointCloud2()
        msg.header.stamp = stamp(self._now)
        msg.header.frame_id = "imu"
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(pts), True
        msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
        self._cloud.publish(msg)
        self._clouds += 1
        if self._clouds in (1, 10) or self._clouds % 100 == 0:
            self.get_logger().info(f"cloud {self._clouds}: {len(pts)} points at t={self._now:.2f}")


def main():
    rclpy.init()
    node = OdinSim(sys.argv[1])
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
