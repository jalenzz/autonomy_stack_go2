#!/usr/bin/env python3
"""Offline regression for traversable terrain filtering on recorded bags.

Replays /registered_scan + /state_estimation through the same geometric rules
as terrain_analysis/traversable_terrain.hpp and scores a forward sector.

Exit code 0 only if stairs/flat bags meet their thresholds.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import sys
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

# ROS2 deserialization (requires unitree_ros2 / workspace env).
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2


TRANSITION_FREE = -1.0


class Params:
    support_band_eps = 0.06
    min_support_points = 3
    wall_bin_size = 0.10
    wall_fill_thre = 0.55
    vehicle_height = 1.5
    seq_length = 4
    flat_eps = 0.08
    min_seq_rise = 0.06
    min_step_rise = 0.10
    max_step_rise = 0.30
    min_significant_steps = 2
    min_width_cells = 2
    width_height_eps = 0.08
    surface_eps = 0.06
    planar_voxel_size = 0.2
    planar_voxel_width = 51
    min_block_points = 10
    quantile_z = 0.25
    obstacle_height_thre = 0.3


def cell_index(width: int, x: int, y: int) -> int:
    return width * x + y


def in_grid(width: int, x: int, y: int) -> bool:
    return 0 <= x < width and 0 <= y < width


def mark_support(
    elevations: np.ndarray,
    point_zs: List[List[float]],
    width: int,
    params: Params,
) -> np.ndarray:
    n = width * width
    support = np.zeros(n, dtype=bool)
    bin_count = max(1, int(math.ceil(params.vehicle_height / params.wall_bin_size)))
    bin_count = min(bin_count, 64)
    for i in range(n):
        zs = point_zs[i]
        if len(zs) < params.min_block_points:
            continue
        elev = float(elevations[i])
        support_count = 0
        bins = [0] * bin_count
        for z in zs:
            if abs(z - elev) <= params.support_band_eps:
                support_count += 1
            above = z - elev
            if 0.0 < above < params.vehicle_height:
                b = int(above / params.wall_bin_size)
                b = min(max(b, 0), bin_count - 1)
                bins[b] = 1
        fill = sum(bins) / float(bin_count)
        if support_count >= params.min_support_points and fill < params.wall_fill_thre:
            support[i] = True
    return support


def sequence_monotonic(
    elevations: np.ndarray, indices: Sequence[int], params: Params
) -> bool:
    if len(indices) < 2:
        return False
    total = float(elevations[indices[-1]] - elevations[indices[0]])
    if abs(total) < params.min_seq_rise:
        return False
    sign = 1 if total > 0 else -1
    steps = 0
    for a, b in zip(indices, indices[1:]):
        d = float(elevations[b] - elevations[a])
        abs_d = abs(d)
        if abs_d <= params.flat_eps:
            continue
        if (1 if d > 0 else -1) != sign:
            return False
        if abs_d > params.max_step_rise:
            return False
        if abs_d < params.min_step_rise:
            continue
        steps += 1
    return steps >= params.min_significant_steps


def has_lateral_width(
    elevations: np.ndarray,
    support: np.ndarray,
    width: int,
    x: int,
    y: int,
    dx: int,
    dy: int,
    params: Params,
) -> bool:
    nx, ny = -dy, dx
    if nx == 0 and ny == 0:
        return False
    if abs(nx) > 1:
        nx = 1 if nx > 0 else -1
    if abs(ny) > 1:
        ny = 1 if ny > 0 else -1
    elev0 = float(elevations[cell_index(width, x, y)])
    width_count = 1
    for s in range(1, params.min_width_cells + 1):
        for sgn in (-1, 1):
            lx = x + sgn * s * nx
            ly = y + sgn * s * ny
            if not in_grid(width, lx, ly):
                continue
            li = cell_index(width, lx, ly)
            if not support[li]:
                continue
            if abs(float(elevations[li]) - elev0) <= params.width_height_eps:
                width_count += 1
    return width_count >= params.min_width_cells


def mark_transition(
    elevations: np.ndarray, support: np.ndarray, width: int, params: Params
) -> np.ndarray:
    n = width * width
    transition = np.zeros(n, dtype=bool)
    dirs = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    L = params.seq_length
    for i in range(n):
        if not support[i]:
            continue
        x, y = divmod(i, width) if False else (i // width, i % width)
        for dx, dy in dirs:
            indices = []
            ok = True
            for s in range(L):
                sx, sy = x + s * dx, y + s * dy
                if not in_grid(width, sx, sy):
                    ok = False
                    break
                si = cell_index(width, sx, sy)
                if not support[si]:
                    ok = False
                    break
                indices.append(si)
            if not ok:
                continue
            if not sequence_monotonic(elevations, indices, params):
                continue
            width_ok = True
            for si in indices:
                sx, sy = si // width, si % width
                if not has_lateral_width(
                    elevations, support, width, sx, sy, dx, dy, params
                ):
                    width_ok = False
                    break
            if not width_ok:
                continue
            for si in indices:
                transition[si] = True
    return transition


def neighbor_elev_range(
    elevations: np.ndarray, support: np.ndarray, width: int, cell: int
) -> Tuple[float, float]:
    x, y = cell // width, cell % width
    lo = hi = float(elevations[cell])
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx, ny = x + dx, y + dy
            if not in_grid(width, nx, ny):
                continue
            ni = cell_index(width, nx, ny)
            if not support[ni]:
                continue
            e = float(elevations[ni])
            lo = min(lo, e)
            hi = max(hi, e)
    return lo, hi


def point_intensity(
    z: float,
    elev: float,
    dis_z: float,
    is_transition: bool,
    elevations: np.ndarray,
    support: np.ndarray,
    width: int,
    cell: int,
    params: Params,
) -> float:
    if not is_transition:
        return dis_z
    if abs(z - elev) <= params.surface_eps:
        return TRANSITION_FREE
    lo, hi = neighbor_elev_range(elevations, support, width, cell)
    if lo - params.surface_eps <= z <= hi + params.surface_eps:
        return TRANSITION_FREE
    return dis_z


def bin_points_to_grid(
    points_xyz: np.ndarray,
    vx: float,
    vy: float,
    params: Params,
) -> Tuple[List[List[float]], List[Tuple[float, float, float, int]]]:
    width = params.planar_voxel_width
    half = (width - 1) // 2
    size = params.planar_voxel_size
    n = width * width
    point_zs: List[List[float]] = [[] for _ in range(n)]
    kept = []
    for p in points_xyz:
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        ind_x = int((x - vx + size / 2) / size) + half
        ind_y = int((y - vy + size / 2) / size) + half
        if (x - vx + size / 2) < 0:
            ind_x -= 1
        if (y - vy + size / 2) < 0:
            ind_y -= 1
        if not in_grid(width, ind_x, ind_y):
            continue
        cell = cell_index(width, ind_x, ind_y)
        point_zs[cell].append(z)
        kept.append((x, y, z, cell))
    return point_zs, kept


def process_grid(
    point_zs: List[List[float]],
    kept: List[Tuple[float, float, float, int]],
    vehicle_xyzyaw: Tuple[float, float, float, float],
    params: Params,
) -> Tuple[np.ndarray, Dict[str, float]]:
    vx, vy, _vz, yaw = vehicle_xyzyaw
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    width = params.planar_voxel_width
    n = width * width

    elevations = np.zeros(n, dtype=np.float32)
    for i in range(n):
        zs = point_zs[i]
        if not zs:
            continue
        zs_sorted = sorted(zs)
        qid = int(params.quantile_z * len(zs_sorted))
        qid = min(max(qid, 0), len(zs_sorted) - 1)
        elevations[i] = zs_sorted[qid]

    support = mark_support(elevations, point_zs, width, params)
    transition = mark_transition(elevations, support, width, params)

    intensities = []
    forward = []
    for x, y, z, cell in kept:
        elev = float(elevations[cell])
        dis_z = z - elev
        if dis_z < 0 or dis_z >= params.vehicle_height:
            continue
        if len(point_zs[cell]) < params.min_block_points:
            continue
        inten = point_intensity(
            z, elev, dis_z, bool(transition[cell]), elevations, support, width, cell, params
        )
        intensities.append(inten)
        dx, dy = x - vx, y - vy
        bx = dx * cos_yaw + dy * sin_yaw
        by = -dx * sin_yaw + dy * cos_yaw
        if 1.0 <= bx <= 3.0 and abs(by) <= bx * math.tan(math.radians(30.0)):
            forward.append(inten)

    arr = np.asarray(intensities, dtype=np.float32) if intensities else np.zeros(0)
    fwd = np.asarray(forward, dtype=np.float32) if forward else np.zeros(0)
    stats = {
        "n_points": float(len(arr)),
        "n_forward": float(len(fwd)),
        "frac_obstacle": float(np.mean(arr > params.obstacle_height_thre)) if len(arr) else 0.0,
        "forward_frac_obstacle": float(np.mean(fwd > params.obstacle_height_thre))
        if len(fwd)
        else 0.0,
        "forward_median": float(np.median(fwd)) if len(fwd) else 0.0,
        "transition_cells": float(np.count_nonzero(transition)),
        "support_cells": float(np.count_nonzero(support)),
    }
    return arr, stats


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_bag_topics(db_path: Path, topics: Sequence[str]):
    con = sqlite3.connect(str(db_path))
    topic_ids = {
        name: con.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
        for name in topics
    }
    for name, tid in topic_ids.items():
        if tid is None:
            raise RuntimeError(f"missing topic {name} in {db_path}")
        topic_ids[name] = tid[0]
    rows = con.execute(
        "SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({}) ORDER BY timestamp".format(
            ",".join("?" for _ in topics)
        ),
        tuple(topic_ids.values()),
    ).fetchall()
    con.close()
    return topic_ids, rows


def evaluate_bag(bag_dir: Path, max_scans: int, params: Params, accum_scans: int = 15) -> Dict:
    db_files = list(bag_dir.glob("*.db3"))
    if not db_files:
        raise RuntimeError(f"no db3 in {bag_dir}")
    db_path = db_files[0]
    topic_ids, rows = load_bag_topics(db_path, ["/registered_scan", "/state_estimation"])
    scan_id = topic_ids["/registered_scan"]
    odom_id = topic_ids["/state_estimation"]

    last_odom = None
    scan_stats = []
    scans_used = 0
    history: List[np.ndarray] = []
    for topic_id, _ts, data in rows:
        if topic_id == odom_id:
            last_odom = deserialize_message(data, Odometry)
            continue
        if topic_id != scan_id or last_odom is None:
            continue
        msg = deserialize_message(data, PointCloud2)
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            continue
        xyz = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        history.append(xyz)
        if len(history) > accum_scans:
            history.pop(0)
        pose = last_odom.pose.pose.position
        yaw = yaw_from_odom(last_odom)
        stacked = np.concatenate(history, axis=0)
        point_zs, kept = bin_points_to_grid(stacked, pose.x, pose.y, params)
        # Score intensity on the latest frame only.
        _, kept_latest = bin_points_to_grid(xyz, pose.x, pose.y, params)
        _, stats = process_grid(
            point_zs, kept_latest, (pose.x, pose.y, pose.z, yaw), params
        )
        scan_stats.append(stats)
        scans_used += 1
        if scans_used >= max_scans:
            break

    if not scan_stats:
        raise RuntimeError(f"no usable scans in {bag_dir}")

    def mean_key(k):
        return float(np.mean([s[k] for s in scan_stats]))

    return {
        "bag": bag_dir.name,
        "scans": scans_used,
        "forward_frac_obstacle": mean_key("forward_frac_obstacle"),
        "frac_obstacle": mean_key("frac_obstacle"),
        "forward_median": mean_key("forward_median"),
        "transition_cells": mean_key("transition_cells"),
        "support_cells": mean_key("support_cells"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bags-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "rosbags",
    )
    parser.add_argument("--max-scans", type=int, default=40)
    parser.add_argument("--stairs-max-forward-frac", type=float, default=0.55)
    parser.add_argument("--stairs-min-transition", type=float, default=5.0)
    parser.add_argument("--flat-min-frac", type=float, default=0.08)
    parser.add_argument("--flat-max-transition", type=float, default=40.0)
    parser.add_argument("--json-out", type=Path, default=None)
    args = parser.parse_args()
    params = Params()

    bags = {
        "stairs": ["stairs_case_1", "stairs_case_2"],
        "flat": ["flat_case_1", "flat_case_2"],
    }
    results = []
    ok = True
    for kind, names in bags.items():
        for name in names:
            bag_dir = args.bags_root / name
            if not bag_dir.exists():
                print(f"MISSING {bag_dir}", file=sys.stderr)
                ok = False
                continue
            summary = evaluate_bag(bag_dir, args.max_scans, params)
            summary["kind"] = kind
            results.append(summary)
            print(json.dumps(summary, sort_keys=True))
            if kind == "stairs":
                if summary["forward_frac_obstacle"] > args.stairs_max_forward_frac:
                    print(
                        f"FAIL {name}: forward_frac_obstacle "
                        f"{summary['forward_frac_obstacle']:.3f} > {args.stairs_max_forward_frac}",
                        file=sys.stderr,
                    )
                    ok = False
                if summary["transition_cells"] < args.stairs_min_transition:
                    print(
                        f"FAIL {name}: transition_cells "
                        f"{summary['transition_cells']:.1f} < {args.stairs_min_transition}",
                        file=sys.stderr,
                    )
                    ok = False
            else:
                if summary["frac_obstacle"] < args.flat_min_frac:
                    print(
                        f"FAIL {name}: frac_obstacle "
                        f"{summary['frac_obstacle']:.3f} < {args.flat_min_frac}",
                        file=sys.stderr,
                    )
                    ok = False
                if summary["transition_cells"] > args.flat_max_transition:
                    print(
                        f"FAIL {name}: transition_cells "
                        f"{summary['transition_cells']:.1f} > {args.flat_max_transition}",
                        file=sys.stderr,
                    )
                    ok = False

    if args.json_out:
        args.json_out.write_text(json.dumps(results, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
