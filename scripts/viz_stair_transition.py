#!/usr/bin/env python3
"""Replay a bag, run traversable-transition detection, publish RViz clouds.

Colored topics:
  /viz/stair_transition  - detected stair/transition points (bright orange RGB)
  /viz/terrain_obstacle  - other terrain points with height residual (intensity)
  /viz/support_surface   - support-cell surface samples (cyan)

Also republishes /registered_scan, /state_estimation, /tf, /tf_static for context.

Example:
  source ~/unitree_ros2/setup.sh && source ~/autonomy_stack_go2/install/setup.bash
  python3 scripts/viz_stair_transition.py --bag rosbags/stairs_case_1
  # other terminal:
  rviz2 -d rviz/stair_transition.rviz
"""

from __future__ import annotations

import argparse
import sqlite3
import struct
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

# Reuse detector from offline eval.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_traversable_terrain import (  # noqa: E402
    Params,
    bin_points_to_grid,
    mark_support,
    mark_transition,
    point_intensity,
    yaw_from_odom,
)

STAIR_RGB = (255, 80, 0)      # orange
SUPPORT_RGB = (0, 220, 255)   # cyan


def rgb_float(r: int, g: int, b: int) -> float:
    return struct.unpack("f", struct.pack("I", (r << 16) | (g << 8) | b))[0]


def make_xyzrgb_cloud(
    points: List[Tuple[float, float, float]],
    rgb: Tuple[int, int, int],
    frame_id: str,
    stamp,
) -> PointCloud2:
    rgb_f = rgb_float(*rgb)
    cloud = [(p[0], p[1], p[2], rgb_f) for p in points]
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(header, fields, cloud)


def make_xyzi_cloud(
    points: List[Tuple[float, float, float, float]],
    frame_id: str,
    stamp,
) -> PointCloud2:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(header, fields, points)


class StairVizPlayer(Node):
    def __init__(self, bag_dir: Path, rate: float, accum_scans: int, loop: bool):
        super().__init__("stair_transition_viz")
        self.bag_dir = bag_dir
        self.rate = max(rate, 0.05)
        self.accum_scans = accum_scans
        self.loop = loop
        self.params = Params()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        tf_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_scan = self.create_publisher(PointCloud2, "/registered_scan", sensor_qos)
        self.pub_odom = self.create_publisher(Odometry, "/state_estimation", 20)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 100)
        self.pub_tf_static = self.create_publisher(TFMessage, "/tf_static", tf_static_qos)
        self.pub_stair = self.create_publisher(PointCloud2, "/viz/stair_transition", 2)
        self.pub_support = self.create_publisher(PointCloud2, "/viz/support_surface", 2)
        self.pub_obstacle = self.create_publisher(PointCloud2, "/viz/terrain_obstacle", 2)

        self.messages = self._load_bag()
        self.index = 0
        self.history: List[np.ndarray] = []
        self.last_odom: Optional[Odometry] = None
        self.tf_static_sent = False
        self.period = 1.0 / self.rate
        self.timer = self.create_timer(0.02, self._tick)
        self.next_pub_time = time.monotonic()
        self.get_logger().info(
            f"Visualizing {bag_dir} ({len(self.messages)} msgs), "
            f"rate={rate}x accum={accum_scans}. Stair marks on /viz/stair_transition"
        )

    def _load_bag(self):
        db_files = list(self.bag_dir.glob("*.db3"))
        if not db_files:
            raise RuntimeError(f"no db3 in {self.bag_dir}")
        con = sqlite3.connect(str(db_files[0]))
        topics = {
            row[0]: (row[1], row[2])
            for row in con.execute("SELECT id, name, type FROM topics").fetchall()
        }
        wanted = {
            "/registered_scan",
            "/state_estimation",
            "/tf",
            "/tf_static",
        }
        rows = con.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
        ).fetchall()
        con.close()
        out = []
        for tid, ts, data in rows:
            name, typ = topics[tid]
            if name in wanted:
                out.append((name, typ, ts, data))
        return out

    def _tick(self):
        now = time.monotonic()
        if now < self.next_pub_time:
            return
        if self.index >= len(self.messages):
            if self.loop:
                self.index = 0
                self.history.clear()
                self.last_odom = None
                self.get_logger().info("loop restart")
            else:
                self.get_logger().info("bag finished")
                self.timer.cancel()
                return

        name, typ, _ts, data = self.messages[self.index]
        self.index += 1

        if name == "/tf_static":
            msg = deserialize_message(data, TFMessage)
            if not self.tf_static_sent:
                self.pub_tf_static.publish(msg)
                self.tf_static_sent = True
            self.next_pub_time = now  # no delay
            return
        if name == "/tf":
            self.pub_tf.publish(deserialize_message(data, TFMessage))
            self.next_pub_time = now
            return
        if name == "/state_estimation":
            self.last_odom = deserialize_message(data, Odometry)
            self.pub_odom.publish(self.last_odom)
            self.next_pub_time = now
            return
        if name != "/registered_scan" or self.last_odom is None:
            self.next_pub_time = now
            return

        scan = deserialize_message(data, PointCloud2)
        self.pub_scan.publish(scan)
        self._publish_detection(scan, self.last_odom)
        # Pace by scan rate roughly.
        self.next_pub_time = now + self.period

    def _publish_detection(self, scan: PointCloud2, odom: Odometry):
        pts = list(pc2.read_points(scan, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            return
        xyz = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        self.history.append(xyz)
        if len(self.history) > self.accum_scans:
            self.history.pop(0)

        pose = odom.pose.pose.position
        yaw = yaw_from_odom(odom)
        stacked = np.concatenate(self.history, axis=0)
        point_zs, _ = bin_points_to_grid(stacked, pose.x, pose.y, self.params)
        _, kept_latest = bin_points_to_grid(xyz, pose.x, pose.y, self.params)

        width = self.params.planar_voxel_width
        n = width * width
        elevations = np.zeros(n, dtype=np.float32)
        for i in range(n):
            zs = point_zs[i]
            if not zs:
                continue
            zs_sorted = sorted(zs)
            qid = int(self.params.quantile_z * len(zs_sorted))
            qid = min(max(qid, 0), len(zs_sorted) - 1)
            elevations[i] = zs_sorted[qid]

        support = mark_support(elevations, point_zs, width, self.params)
        transition = mark_transition(elevations, support, width, self.params)

        stair_pts = []
        support_pts = []
        obstacle_pts = []
        for x, y, z, cell in kept_latest:
            elev = float(elevations[cell])
            dis_z = z - elev
            if dis_z < 0 or dis_z >= self.params.vehicle_height:
                continue
            if len(point_zs[cell]) < self.params.min_block_points:
                continue
            inten = point_intensity(
                z,
                elev,
                dis_z,
                bool(transition[cell]),
                elevations,
                support,
                width,
                cell,
                self.params,
            )
            if transition[cell] and inten < 0.0:
                stair_pts.append((x, y, z))
            elif support[cell] and abs(z - elev) <= self.params.surface_eps:
                support_pts.append((x, y, z))
            elif inten > self.params.obstacle_height_thre:
                obstacle_pts.append((x, y, z, float(inten)))

        stamp = scan.header.stamp
        frame = scan.header.frame_id or "camera_init"
        self.pub_stair.publish(make_xyzrgb_cloud(stair_pts, STAIR_RGB, frame, stamp))
        self.pub_support.publish(make_xyzrgb_cloud(support_pts, SUPPORT_RGB, frame, stamp))
        self.pub_obstacle.publish(make_xyzi_cloud(obstacle_pts, frame, stamp))

        if self.index % 20 == 0:
            self.get_logger().info(
                f"stair={len(stair_pts)} support={len(support_pts)} "
                f"obstacle={len(obstacle_pts)} transition_cells={int(transition.sum())}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "rosbags" / "stairs_case_1",
    )
    parser.add_argument("--rate", type=float, default=2.0, help="scan playback multiplier pace")
    parser.add_argument("--accum-scans", type=int, default=15)
    parser.add_argument("--loop", action="store_true")
    args = parser.parse_args()
    if not args.bag.exists():
        print(f"bag not found: {args.bag}", file=sys.stderr)
        return 1

    rclpy.init()
    node = StairVizPlayer(args.bag, args.rate, args.accum_scans, args.loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
