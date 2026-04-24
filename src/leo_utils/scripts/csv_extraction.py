#!/usr/bin/env python3
"""
Session Metrics Extractor — no ROS installation required.
Uses rosbags library for bag reading/deserialization.

Usage:
    python3 extract_metrics.py <session_folder>

    <session_folder> should contain subfolders named:
        session_yamauchi, session_gao, session_rss_1, ..., session_rss_4

Output:
    <session_folder>/metrics.csv

Dependencies:
    pip install rosbags numpy
"""

import sys
import os
import re
import glob
import json
import csv
import argparse

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# ── Configuration ──────────────────────────────────────────────────────────────

SESSION_SUBFOLDERS = [
    "session_yamauchi",
    "session_gao",
    "session_rss_1",
    "session_rss_2",
    "session_rss_3",
    "session_rss_4",
]

COMBINATIONS = [
    (-19.0, -19.0,  19.0, -19.0),
    (-19.0, -19.0,  -8.0, -18.0),
    (-19.0, -19.0, -19.0,  19.0),
    (-19.0, -19.0,  18.0,  18.0),
    (-19.0, -19.0,   8.0,   0.0),
    ( 19.0, -19.0, -19.0, -19.0),
    ( 19.0, -19.0,  -8.0, -18.0),
    ( 19.0, -19.0, -19.0,  19.0),
    ( 19.0, -19.0,  18.0,  18.0),
    ( 19.0, -19.0,   8.0,   0.0),
    (  0.0,   0.0, -19.0, -19.0),
    (  0.0,   0.0,  19.0, -19.0),
    (  0.0,   0.0,  -8.0, -18.0),
    (  0.0,   0.0, -19.0,  19.0),
    (  0.0,   0.0,  18.0,  18.0),
    (  0.0,   0.0,   8.0,   0.0),
    (-19.0,  19.0, -19.0, -19.0),
    (-19.0,  19.0,  19.0, -19.0),
    (-19.0,  19.0,  -8.0, -18.0),
    (-19.0,  19.0,  18.0,  18.0),
    (-19.0,  19.0,   8.0,   0.0),
    ( 19.0,  19.0, -19.0, -19.0),
    ( 19.0,  19.0,  19.0, -19.0),
    ( 19.0,  19.0,  -8.0, -18.0),
    ( 19.0,  19.0, -19.0,  19.0),
    ( 19.0,  19.0,   8.0,   0.0),
]

CSV_COLUMNS = [
    "bag_name",
    "mode",
    "stddev",
    "combination_number",
    "robot_start_x",
    "robot_start_y",
    "router_x",
    "router_y",
    "initial_distance",
    "termination_reason",
    "success",
    "duration_s",
    "distance_travelled_m",
    "map_size_final_m2",
    "goals_published",
    "time_to_half",
    "time_to_75pct",
    "time_to_90pct",
    "closest_distance",
    "time_at_closest",
    "final_distance",
    "gradient_mean_error_deg",
    "gradient_median_error_deg",
    "gradient_n_samples",
]

# ── Bag reading ────────────────────────────────────────────────────────────────

def read_topics(bag_path, topic_names):
    """
    Read messages from a rosbag2 directory using rosbags (no ROS required).
    Returns dict: topic_name -> list of (timestamp_ns, msg)
    """
    data = {t: [] for t in topic_names}
    wanted = set(topic_names)

    try:
        with Reader(bag_path) as reader:
            # Build map of topic name -> msgtype string
            type_map = {}
            for conn in reader.connections:
                if conn.topic in wanted:
                    type_map[conn.topic] = conn.msgtype

            for topic in wanted:
                if topic not in type_map:
                    print(f"  WARNING: '{topic}' not in bag, skipping")

            for conn, timestamp, rawdata in reader.messages():
                if conn.topic not in wanted:
                    continue
                try:
                    msg = deserialize_cdr(rawdata, conn.msgtype)
                    data[conn.topic].append((timestamp, msg))
                except Exception as e:
                    pass

    except Exception as e:
        print(f"  ERROR opening bag {bag_path}: {e}")

    return data


# ── TF: compose map->odom->base_link ──────────────────────────────────────────

def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)]
    ])


def compose_transforms(t1_xyz, q1, t2_xyz, q2):
    R1    = quat_to_mat(q1)
    R2    = quat_to_mat(q2)
    t_out = R1 @ np.array(t2_xyz) + np.array(t1_xyz)
    R_out = R1 @ R2
    tr    = R_out[0,0] + R_out[1,1] + R_out[2,2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R_out[2,1] - R_out[1,2]) * s
        y = (R_out[0,2] - R_out[2,0]) * s
        z = (R_out[1,0] - R_out[0,1]) * s
    else:
        w, x, y, z = 1, 0, 0, 0
    return t_out, np.array([x, y, z, w])


def extract_robot_trajectory(tf_messages):
    """
    Compose map->odom and odom->base_link to get map->base_link.
    Returns array of (t_sec, x, y, yaw, roll_deg, pitch_deg) sorted by time.
    """
    map_odom  = []
    odom_base = []

    for ts, msg in tf_messages:
        for tf in msg.transforms:
            p = tf.transform.translation
            r = tf.transform.rotation
            entry = (ts, p.x, p.y, p.z, r.x, r.y, r.z, r.w)
            if tf.header.frame_id == 'map'  and tf.child_frame_id == 'odom':
                map_odom.append(entry)
            elif tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                odom_base.append(entry)

    if not map_odom or not odom_base:
        print("  WARNING: Could not find map->odom or odom->base_link transforms")
        return np.array([]).reshape(0, 6)

    map_odom.sort(key=lambda x: x[0])
    odom_base.sort(key=lambda x: x[0])
    mo_ts = np.array([e[0] for e in map_odom])

    trajectory = []
    for ob in odom_base:
        ts  = ob[0]
        idx = min(np.searchsorted(mo_ts, ts), len(map_odom) - 1)
        mo  = map_odom[idx]

        t_out, q_out = compose_transforms(
            [mo[1], mo[2], mo[3]], [mo[4], mo[5], mo[6], mo[7]],
            [ob[1], ob[2], ob[3]], [ob[4], ob[5], ob[6], ob[7]]
        )
        x, y, z, w = q_out
        yaw   = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        trajectory.append((ts / 1e9, t_out[0], t_out[1], yaw,
                           np.degrees(roll), np.degrees(pitch)))

    trajectory.sort(key=lambda x: x[0])
    return np.array(trajectory[::5])  # downsample


def interpolate_pose(trajectory, timestamps):
    traj_t = trajectory[:, 0]
    x_interp = np.interp(timestamps, traj_t, trajectory[:, 1])
    y_interp = np.interp(timestamps, traj_t, trajectory[:, 2])
    return x_interp, y_interp


# ── Map coverage ───────────────────────────────────────────────────────────────

def map_coverage_over_time(map_messages):
    times, coverage = [], []
    for ts, msg in map_messages:
        data  = np.array(msg.data)
        known = np.sum(data >= 0)
        area  = known * msg.info.resolution ** 2
        times.append(ts / 1e9)
        coverage.append(area)
    return np.array(times), np.array(coverage)


# ── Gradient ───────────────────────────────────────────────────────────────────

def extract_gradient_over_time(grad_messages):
    result = []
    for ts, msg in grad_messages:
        gx, gy = msg.vector.x, msg.vector.y
        mag    = np.sqrt(gx**2 + gy**2)
        if mag > 1e-6:
            result.append((ts / 1e9, gx / mag, gy / mag, mag))
        else:
            result.append((ts / 1e9, 0.0, 0.0, 0.0))

    if not result:
        return result

    mags   = np.array([r[3] for r in result])
    median = np.median(mags[mags > 1e-6]) if np.any(mags > 1e-6) else 1.0
    result = [r for r in result if r[3] <= 10.0 * median]
    return result


# ── Goals ──────────────────────────────────────────────────────────────────────

def extract_goals(goal_messages):
    return [(ts / 1e9, msg.pose.position.x, msg.pose.position.y)
            for ts, msg in goal_messages]


# ── Metrics helpers ────────────────────────────────────────────────────────────

def compute_path_length(trajectory):
    if len(trajectory) < 2:
        return 0.0
    pts = trajectory[:, 1:3]
    return float(np.sum(np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1))))


def distance_to_router(trajectory, rx, ry):
    dx = trajectory[:, 1] - rx
    dy = trajectory[:, 2] - ry
    return trajectory[:, 0], np.sqrt(dx**2 + dy**2)


def angle_diff(a, b):
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi


def compute_milestones(dist_times, dist_arr, initial_distance):
    nan = float('nan')
    if len(dist_arr) == 0:
        return nan, nan, nan, nan, nan, nan

    thresholds = {
        'half':  initial_distance * 0.50,
        '75pct': initial_distance * 0.25,
        '90pct': initial_distance * 0.10,
    }
    results = {}
    for label, threshold in thresholds.items():
        idx = np.where(dist_arr <= threshold)[0]
        results[label] = float(dist_times[idx[0]]) if len(idx) > 0 else nan

    closest_idx      = int(np.argmin(dist_arr))
    closest_distance = float(dist_arr[closest_idx])
    time_at_closest  = float(dist_times[closest_idx])
    final_distance   = float(dist_arr[-1])

    return (results['half'], results['75pct'], results['90pct'],
            closest_distance, time_at_closest, final_distance)


def compute_gradient_error(grad_messages, trajectory, router_x, router_y):
    nan = float('nan')
    if not grad_messages or len(trajectory) < 2:
        return nan, nan, 0

    gradients = extract_gradient_over_time(grad_messages)
    if not gradients:
        return nan, nan, 0

    grad_arr = np.array(gradients)
    gt       = grad_arr[:, 0]

    robot_x, robot_y = interpolate_pose(trajectory, gt)
    true_bearing     = np.arctan2(router_y - robot_y, router_x - robot_x)
    grad_bearing     = np.arctan2(grad_arr[:, 2], grad_arr[:, 1])
    errors           = np.abs(np.degrees(
        np.array([angle_diff(g, t) for g, t in zip(grad_bearing, true_bearing)])
    ))

    return (round(float(np.mean(errors)), 2),
            round(float(np.median(errors)), 2),
            len(errors))


# ── Name parsers ───────────────────────────────────────────────────────────────

def parse_router_from_name(bag_path):
    name  = os.path.basename(bag_path.rstrip('/'))
    match = re.search(r'_rx([-\d.]+)_ry([-\d.]+)_bx', name)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None, None


def parse_robot_start(bag_name):
    match = re.search(r'_bx([-\d.]+)_by([-\d.]+)', bag_name)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None, None


def infer_mode_stddev(subfolder_name):
    name = subfolder_name.lower()
    if 'yamauchi' in name:
        return 'yamauchi', 0
    if 'gao' in name:
        return 'gao', 0
    match = re.search(r'rss_(\d+)', name)
    if match:
        return 'rss', int(match.group(1))
    return 'unknown', -1


def lookup_combination_number(bx, by, rx, ry):
    for i, (cbx, cby, crx, cry) in enumerate(COMBINATIONS, start=1):
        if bx == cbx and by == cby and rx == crx and ry == cry:
            return i
    return -1


def read_result_json(bag_path):
    result_path = os.path.join(bag_path, 'result.json')
    if not os.path.exists(result_path):
        return {}
    try:
        with open(result_path) as f:
            return json.load(f)
    except Exception:
        return {}


# ── Per-bag extraction ─────────────────────────────────────────────────────────

def extract_metrics(bag_path, mode, stddev):
    bag_name = os.path.basename(bag_path.rstrip('/'))
    print(f"    {bag_name}")

    router_x, router_y         = parse_router_from_name(bag_path)
    robot_start_x, robot_start_y = parse_robot_start(bag_name)

    if router_x is None or robot_start_x is None:
        print(f"      ERROR: Could not parse positions, skipping.")
        return None

    combination_number = lookup_combination_number(
        robot_start_x, robot_start_y, router_x, router_y
    )
    if combination_number == -1:
        print(f"      WARNING: Position combination not found in lookup table.")

    initial_distance = float(np.sqrt(
        (robot_start_x - router_x)**2 + (robot_start_y - router_y)**2
    ))

    result             = read_result_json(bag_path)
    termination_reason = result.get('termination_reason', 'unknown')
    success            = 'yes' if termination_reason == 'router_found' else 'no'

    topics_needed = ['/tf', '/map', '/cmd_vel', '/goal_pose', '/rss_gradient']
    try:
        data = read_topics(bag_path, topics_needed)
    except Exception as e:
        print(f"      ERROR reading bag: {e}")
        return None

    trajectory = extract_robot_trajectory(data['/tf'])
    if len(trajectory) < 2:
        print(f"      WARNING: Not enough trajectory points, skipping.")
        return None

    t0              = trajectory[0, 0]
    trajectory[:, 0] -= t0
    duration        = float(trajectory[-1, 0])

    _, cov          = map_coverage_over_time(data['/map'])
    map_size_final  = float(cov[-1]) if len(cov) > 0 else 0.0

    distance_travelled = compute_path_length(trajectory)
    goals_published    = len(extract_goals(data['/goal_pose']))

    dist_times, dist_arr = distance_to_router(trajectory, router_x, router_y)

    time_to_half, time_to_75pct, time_to_90pct, \
    closest_distance, time_at_closest, final_distance = \
        compute_milestones(dist_times, dist_arr, initial_distance)

    nan = float('nan')
    if mode == 'rss' and data['/rss_gradient']:
        grad_mean_err, grad_median_err, grad_n = compute_gradient_error(
            data['/rss_gradient'], trajectory, router_x, router_y
        )
    else:
        grad_mean_err, grad_median_err, grad_n = nan, nan, 0

    return {
        "bag_name":                  bag_name,
        "mode":                      mode,
        "stddev":                    stddev,
        "combination_number":        combination_number,
        "robot_start_x":             robot_start_x,
        "robot_start_y":             robot_start_y,
        "router_x":                  router_x,
        "router_y":                  router_y,
        "initial_distance":          round(initial_distance, 3),
        "termination_reason":        termination_reason,
        "success":                   success,
        "duration_s":                round(duration, 2),
        "distance_travelled_m":      round(distance_travelled, 3),
        "map_size_final_m2":         round(map_size_final, 2),
        "goals_published":           goals_published,
        "time_to_half":              round(time_to_half, 2) if not np.isnan(time_to_half) else nan,
        "time_to_75pct":             round(time_to_75pct, 2) if not np.isnan(time_to_75pct) else nan,
        "time_to_90pct":             round(time_to_90pct, 2) if not np.isnan(time_to_90pct) else nan,
        "closest_distance":          round(closest_distance, 3),
        "time_at_closest":           round(time_at_closest, 2),
        "final_distance":            round(final_distance, 3),
        "gradient_mean_error_deg":   grad_mean_err,
        "gradient_median_error_deg": grad_median_err,
        "gradient_n_samples":        grad_n,
    }


# ── CSV helpers ────────────────────────────────────────────────────────────────

def load_existing_bag_names(csv_path):
    if not os.path.exists(csv_path):
        return set()
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        return {row["bag_name"] for row in reader}


def append_rows(csv_path, rows):
    file_exists = os.path.exists(csv_path)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        if not file_exists:
            writer.writeheader()
        writer.writerows(rows)


# ── Session walker ─────────────────────────────────────────────────────────────

def process_session(session_folder):
    csv_path = os.path.join(session_folder, "metrics.csv")

    print(f"\n{'='*60}")
    print(f"Session folder: {session_folder}")
    print(f"Output CSV:     {csv_path}")
    print(f"{'='*60}")

    already_processed = load_existing_bag_names(csv_path)
    print(f"Already in CSV: {len(already_processed)} runs.")

    new_rows = []
    skipped  = 0
    failed   = 0

    for subfolder_name in SESSION_SUBFOLDERS:
        subfolder_path = os.path.join(session_folder, subfolder_name)

        if not os.path.isdir(subfolder_path):
            print(f"\n  WARNING: Subfolder not found: {subfolder_path}")
            continue

        mode, stddev = infer_mode_stddev(subfolder_name)
        print(f"\n  [{subfolder_name}] mode={mode}, stddev={stddev}")

        bag_dirs = sorted([
            d for d in glob.glob(os.path.join(subfolder_path, '*/'))
            if glob.glob(os.path.join(d, '*.db3'))
        ])

        if not bag_dirs:
            print(f"    WARNING: No bags found in {subfolder_path}")
            continue

        print(f"    Found {len(bag_dirs)} bags.")

        for bag_path in bag_dirs:
            bag_name = os.path.basename(bag_path.rstrip('/'))

            if bag_name in already_processed:
                skipped += 1
                continue

            row = extract_metrics(bag_path, mode, stddev)
            if row is None:
                failed += 1
                continue

            new_rows.append(row)
            print(f"      -> {row['termination_reason']}, "
                  f"comb={row['combination_number']}, "
                  f"dur={row['duration_s']}s, "
                  f"closest={row['closest_distance']}m")

    if new_rows:
        append_rows(csv_path, new_rows)
        print(f"\nAppended {len(new_rows)} new rows to {csv_path}")
    else:
        print("\nNo new rows to add.")

    print(f"Skipped: {skipped} | Failed: {failed} | Added: {len(new_rows)}")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Extract metrics from a simulation session folder into a CSV."
    )
    parser.add_argument("session_folder", help="Path to session folder containing session_yamauchi, session_gao, etc.")
    args = parser.parse_args()

    if not os.path.isdir(args.session_folder):
        print(f"ERROR: Not a directory: {args.session_folder}")
        sys.exit(1)

    process_session(args.session_folder)
    print("\nDone.")


if __name__ == '__main__':
    main()