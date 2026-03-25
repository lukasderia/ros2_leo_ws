#!/usr/bin/env python3
"""
Real Bag Metrics Extractor
Walks bags_real/session_* folders, reads info.json for manual metadata,
extracts per-run metrics from each bag, and writes a single CSV file.

Usage:
    python3 extract_metrics_real.py

Requirements:
    source /opt/ros/foxy/setup.bash
    source ~/ros2_leo_ws/install/setup.bash

Dependencies: bag_analysis.py must be in the same directory.
"""

import sys
import os
import re
import glob
import json
import csv

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_analysis import (
    read_topics,
    extract_robot_trajectory,
    map_coverage_over_time,
    extract_gradient_over_time,
    extract_goals,
    compute_path_length,
    distance_to_router,
    interpolate_pose,
    angle_diff,
)

# ── Configuration ─────────────────────────────────────────────────────────────

REAL_BAGS_ROOT = "/media/lukas/VERBATIM HD/Lukas Master/bags_real"

CSV_OUTPUT = "/media/lukas/VERBATIM HD/Lukas Master/metrics_real.csv"

SESSION_SUBFOLDERS = [
    "session_yamauchi",
    "session_gao",
    "session_rss",
]

# ── CSV schema (same as simulation where applicable) ──────────────────────────

CSV_COLUMNS = [
    # Identity
    "bag_name",
    "mode",
    "stddev",           # always 0 for real runs
    "blacklist",        # always None for real runs
    "combination_number",  # always -1 for real runs
    "robot_start_x",
    "robot_start_y",
    "router_x",
    "router_y",
    "initial_distance",
    # Termination
    "termination_reason",
    "success",
    "duration_s",
    # Exploration
    "distance_travelled_m",
    "map_size_final_m2",
    "goals_published",
    # Router approach milestones (NaN if not reached)
    "time_to_half",
    "time_to_75pct",
    "time_to_90pct",
    "closest_distance",
    "time_at_closest",
    "final_distance",
    # Gradient quality (RSS only, NaN otherwise)
    "gradient_mean_error_deg",
    "gradient_median_error_deg",
    "gradient_n_samples",
]

# ── Info JSON reader ──────────────────────────────────────────────────────────

def read_info_json(bag_path):
    info_path = os.path.join(bag_path, 'info.json')
    if not os.path.exists(info_path):
        return None
    try:
        with open(info_path) as f:
            return json.load(f)
    except Exception as e:
        print(f"      ERROR reading info.json: {e}")
        return None

# ── Milestone computation (identical to simulation script) ────────────────────

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

    return (
        results['half'],
        results['75pct'],
        results['90pct'],
        closest_distance,
        time_at_closest,
        final_distance,
    )

# ── Gradient error computation (identical to simulation script) ───────────────

def compute_gradient_error(grad_messages, trajectory, router_x, router_y):
    nan = float('nan')

    if not grad_messages or len(trajectory) < 2:
        return nan, nan, 0

    gradients = extract_gradient_over_time(grad_messages)
    if not gradients:
        return nan, nan, 0

    grad_arr = np.array(gradients)          # (N, 4): t, gx_norm, gy_norm, mag
    gt       = grad_arr[:, 0]

    robot_x, robot_y = interpolate_pose(trajectory, gt)

    true_bearing = np.arctan2(router_y - robot_y, router_x - robot_x)
    grad_bearing = np.arctan2(grad_arr[:, 2], grad_arr[:, 1])

    errors = np.abs(np.degrees(
        np.array([angle_diff(g, t) for g, t in zip(grad_bearing, true_bearing)])
    ))

    return (
        round(float(np.mean(errors)),   2),
        round(float(np.median(errors)), 2),
        len(errors),
    )

# ── Per-bag extraction ────────────────────────────────────────────────────────

def extract_metrics(bag_path, mode, router_x, router_y, termination_reason):
    bag_name = os.path.basename(bag_path.rstrip('/'))
    print(f"    {bag_name}")

    topics_needed = ['/tf', '/map', '/cmd_vel', '/goal_pose', '/rss_gradient']
    try:
        data = read_topics(bag_path, topics_needed)
    except Exception as e:
        print(f"      ERROR reading bag: {e}")
        return None

    # Trajectory
    trajectory = extract_robot_trajectory(data['/tf'])
    if len(trajectory) < 2:
        print(f"      WARNING: Not enough trajectory points, skipping.")
        return None

    # Normalize time to t=0
    t0 = trajectory[0, 0]
    trajectory[:, 0] -= t0

    # Robot start in original map coordinates (before shift)
    robot_start_x = float(trajectory[0, 1])
    robot_start_y = float(trajectory[0, 2])

    # Shift trajectory so robot starts at (0, 0)
    trajectory[:, 1] -= robot_start_x
    trajectory[:, 2] -= robot_start_y

    # Shift router position by the same amount so it stays correct
    # relative to the shifted trajectory
    router_x -= robot_start_x
    router_y -= robot_start_y

    # Duration
    duration = float(trajectory[-1, 0])

    # Map coverage
    _, cov = map_coverage_over_time(data['/map'])
    map_size_final = float(cov[-1]) if len(cov) > 0 else 0.0

    # Navigation
    distance_travelled = compute_path_length(trajectory)

    # Goals
    goals_published = len(extract_goals(data['/goal_pose']))

    # Initial distance (router position is relative to robot start)
    initial_distance = float(np.sqrt(router_x**2 + router_y**2))

    # Distance to router time series
    dist_times, dist_arr = distance_to_router(trajectory, router_x, router_y)

    # Milestones
    time_to_half, time_to_75pct, time_to_90pct, \
    closest_distance, time_at_closest, final_distance = \
        compute_milestones(dist_times, dist_arr, initial_distance)

    # Gradient error (RSS only)
    nan = float('nan')
    if mode == 'rss' and data['/rss_gradient']:
        grad_mean_err, grad_median_err, grad_n = compute_gradient_error(
            data['/rss_gradient'], trajectory, router_x, router_y
        )
    else:
        grad_mean_err, grad_median_err, grad_n = nan, nan, 0

    success = 'yes' if termination_reason == 'router_found' else 'no'

    return {
        "bag_name":                  bag_name,
        "mode":                      mode,
        "stddev":                    0,
        "blacklist":                 None,
        "combination_number":        -1,
        "robot_start_x":             round(robot_start_x, 3),
        "robot_start_y":             round(robot_start_y, 3),
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

# ── CSV helpers ───────────────────────────────────────────────────────────────

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

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print(f"Real bags root: {REAL_BAGS_ROOT}")
    print(f"Output CSV:     {CSV_OUTPUT}")

    already_processed = load_existing_bag_names(CSV_OUTPUT)
    print(f"Already in CSV: {len(already_processed)} runs.\n")

    new_rows = []
    skipped  = 0
    failed   = 0
    total    = 0

    for subfolder_name in SESSION_SUBFOLDERS:
        subfolder_path = os.path.join(REAL_BAGS_ROOT, subfolder_name)

        if not os.path.isdir(subfolder_path):
            print(f"WARNING: Subfolder not found: {subfolder_path}")
            continue

        print(f"[{subfolder_name}]")

        bag_dirs = sorted([
            d for d in glob.glob(os.path.join(subfolder_path, '*/'))
            if glob.glob(os.path.join(d, '*.db3'))
        ])

        if not bag_dirs:
            print(f"  WARNING: No bags found.")
            continue

        total += len(bag_dirs)
        print(f"  Found {len(bag_dirs)} bags.")

        for bag_path in bag_dirs:
            bag_name = os.path.basename(bag_path.rstrip('/'))

            if bag_name in already_processed:
                skipped += 1
                continue

            info = read_info_json(bag_path)
            if info is None:
                print(f"    ERROR: No info.json in {bag_name}, skipping.")
                failed += 1
                continue

            mode               = info.get('mode', 'unknown')
            router_x           = float(info.get('router_x', 0.0))
            router_y           = float(info.get('router_y', 0.0))
            termination_reason = info.get('termination_reason', 'unknown')

            row = extract_metrics(bag_path, mode, router_x, router_y, termination_reason)
            if row is None:
                failed += 1
                continue

            new_rows.append(row)
            print(f"      -> {row['termination_reason']}, "
                  f"dur={row['duration_s']}s, "
                  f"closest={row['closest_distance']}m, "
                  f"mode={row['mode']}")

    if new_rows:
        append_rows(CSV_OUTPUT, new_rows)
        print(f"\nAppended {len(new_rows)} new rows to {CSV_OUTPUT}")
    else:
        print("\nNo new rows to add.")

    print(f"\nTotal bags: {total} | Skipped: {skipped} | Failed: {failed} | Added: {len(new_rows)}")


if __name__ == '__main__':
    main()