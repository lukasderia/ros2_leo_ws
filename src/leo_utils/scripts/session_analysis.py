#!/usr/bin/env python3
"""
Session Metrics Extractor
Walks both simulation session sets (blacklist / no-blacklist),
extracts per-run metrics from each bag, and writes two CSV files.

Usage:
    python3 extract_metrics.py

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
import argparse

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_analysis import (
    read_topics,
    extract_robot_trajectory,
    map_coverage_over_time,
    extract_gradient_over_time,
    extract_velocity,
    extract_goals,
    compute_path_length,
    distance_to_router,
    interpolate_pose,
    angle_diff,
    parse_router_from_name,
)

# ── Configuration ─────────────────────────────────────────────────────────────

SET_BLACKLIST    = "/media/lukas/VERBATIM HD/Lukas Master/Session_blacklist"
SET_NO_BLACKLIST = "/media/lukas/VERBATIM HD/Lukas Master/Session_no_blacklist"

CSV_BLACKLIST    = "/media/lukas/VERBATIM HD/Lukas Master/metrics_blacklist.csv"
CSV_NO_BLACKLIST = "/media/lukas/VERBATIM HD/Lukas Master/metrics_no_blacklist.csv"

# All 6 session subfolders expected inside each set
SESSION_SUBFOLDERS = [
    "session_yamauchi",
    "session_gao",
    "session_rss_1",
    "session_rss_2",
    "session_rss_3",
    "session_rss_4",
]

# Position combinations numbered 1-26 (bx, by, rx, ry)
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

# ── CSV schema ────────────────────────────────────────────────────────────────

CSV_COLUMNS = [
    # Identity
    "bag_name",
    "mode",
    "stddev",
    "blacklist",
    "combination_number",
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
    # Gradient quality (RSS modes only, NaN otherwise)
    "gradient_mean_error_deg",
    "gradient_median_error_deg",
    "gradient_n_samples",
]

# ── Name / folder parsers ─────────────────────────────────────────────────────

def parse_robot_start(bag_name):
    match = re.search(r'_bx([-\d.]+)_by([-\d.]+)', bag_name)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None, None


def infer_mode_stddev(subfolder_name):
    """
    Returns (mode, stddev) from subfolder name.
    session_yamauchi -> ('yamauchi', 0)
    session_gao      -> ('gao', 0)
    session_rss_1    -> ('rss', 1)
    """
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
    return -1  # not found


def read_result_json(bag_path):
    result_path = os.path.join(bag_path, 'result.json')
    if not os.path.exists(result_path):
        return {}
    try:
        with open(result_path) as f:
            return json.load(f)
    except Exception:
        return {}

# ── Milestone computation ─────────────────────────────────────────────────────

def compute_milestones(dist_times, dist_arr, initial_distance):
    """
    Given distance time series and initial distance, return:
    time_to_half, time_to_75pct, time_to_90pct,
    closest_distance, time_at_closest, final_distance
    
    Milestones are defined as proportional reduction in distance:
    - half:   distance <= initial * 0.50
    - 75pct:  distance <= initial * 0.25  (75% of the way there)
    - 90pct:  distance <= initial * 0.10  (90% of the way there)
    
    Returns NaN for milestones not reached.
    """
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

    closest_idx = int(np.argmin(dist_arr))
    closest_distance   = float(dist_arr[closest_idx])
    time_at_closest    = float(dist_times[closest_idx])
    final_distance     = float(dist_arr[-1])

    return (
        results['half'],
        results['75pct'],
        results['90pct'],
        closest_distance,
        time_at_closest,
        final_distance,
    )

# ── Gradient error computation ────────────────────────────────────────────────

def compute_gradient_error(grad_messages, trajectory, router_x, router_y):
    """
    For each gradient sample, compute angular error (degrees) between
    estimated gradient direction and true bearing to router.
    Returns (mean_error, median_error, n_samples) or (NaN, NaN, 0).
    """
    nan = float('nan')

    if not grad_messages or len(trajectory) < 2:
        return nan, nan, 0

    gradients = extract_gradient_over_time(grad_messages)
    if not gradients:
        return nan, nan, 0

    grad_arr = np.array(gradients)          # (N, 4): t, gx_norm, gy_norm, mag
    gt       = grad_arr[:, 0]

    # Interpolate robot position at each gradient timestamp
    robot_x, robot_y = interpolate_pose(trajectory, gt)

    # True bearing from robot to router
    true_bearing = np.arctan2(router_y - robot_y, router_x - robot_x)

    # Estimated bearing from gradient direction
    grad_bearing = np.arctan2(grad_arr[:, 2], grad_arr[:, 1])

    # Absolute angular error in degrees
    errors = np.abs(np.degrees(
        np.array([angle_diff(g, t) for g, t in zip(grad_bearing, true_bearing)])
    ))

    return (
        round(float(np.mean(errors)),   2),
        round(float(np.median(errors)), 2),
        len(errors),
    )

# ── Per-bag extraction ────────────────────────────────────────────────────────

def extract_metrics(bag_path, mode, stddev, blacklist):
    bag_name = os.path.basename(bag_path.rstrip('/'))
    print(f"    {bag_name}")

    # Parse positions from bag name
    router_x, router_y   = parse_router_from_name(bag_path)
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

    # Read result.json for termination info
    result = read_result_json(bag_path)
    termination_reason = result.get('termination_reason', 'unknown')
    success = 'yes' if termination_reason == 'router_found' else 'no'

    # Read bag topics
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

    # Duration
    duration = float(trajectory[-1, 0])

    # Map coverage
    _, cov = map_coverage_over_time(data['/map'])
    map_size_final = float(cov[-1]) if len(cov) > 0 else 0.0

    # Navigation
    distance_travelled = compute_path_length(trajectory)

    # Goals
    goals_published = len(extract_goals(data['/goal_pose']))

    # Distance to router time series
    dist_times, dist_arr = distance_to_router(trajectory, router_x, router_y)
    # Normalize dist_times to t=0 (trajectory already normalized, dist_times comes from it)
    
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

    return {
        "bag_name":                 bag_name,
        "mode":                     mode,
        "stddev":                   stddev,
        "blacklist":                blacklist,
        "combination_number":       combination_number,
        "robot_start_x":            robot_start_x,
        "robot_start_y":            robot_start_y,
        "router_x":                 router_x,
        "router_y":                 router_y,
        "initial_distance":         round(initial_distance, 3),
        "termination_reason":       termination_reason,
        "success":                  success,
        "duration_s":               round(duration, 2),
        "distance_travelled_m":     round(distance_travelled, 3),
        "map_size_final_m2":        round(map_size_final, 2),
        "goals_published":          goals_published,
        "time_to_half":             round(time_to_half, 2) if not np.isnan(time_to_half) else nan,
        "time_to_75pct":            round(time_to_75pct, 2) if not np.isnan(time_to_75pct) else nan,
        "time_to_90pct":            round(time_to_90pct, 2) if not np.isnan(time_to_90pct) else nan,
        "closest_distance":         round(closest_distance, 3),
        "time_at_closest":          round(time_at_closest, 2),
        "final_distance":           round(final_distance, 3),
        "gradient_mean_error_deg":  grad_mean_err,
        "gradient_median_error_deg":grad_median_err,
        "gradient_n_samples":       grad_n,
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

# ── Session walker ────────────────────────────────────────────────────────────

def process_set(set_folder, csv_path, blacklist):
    print(f"\n{'='*60}")
    print(f"Processing set: {set_folder}")
    print(f"Blacklist: {blacklist}")
    print(f"Output CSV: {csv_path}")
    print(f"{'='*60}")

    already_processed = load_existing_bag_names(csv_path)
    print(f"Already in CSV: {len(already_processed)} runs.")

    new_rows = []
    skipped  = 0
    failed   = 0

    for subfolder_name in SESSION_SUBFOLDERS:
        subfolder_path = os.path.join(set_folder, subfolder_name)

        if not os.path.isdir(subfolder_path):
            print(f"\n  WARNING: Subfolder not found: {subfolder_path}")
            continue

        mode, stddev = infer_mode_stddev(subfolder_name)
        print(f"\n  [{subfolder_name}] mode={mode}, stddev={stddev}")

        # Find all bag directories (contain a .db3 file)
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

            row = extract_metrics(bag_path, mode, stddev, blacklist)
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
    return len(new_rows), failed

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Extract metrics from both simulation sets.")
    parser.add_argument("--blacklist-only",    action="store_true", help="Only process blacklist set")
    parser.add_argument("--no-blacklist-only", action="store_true", help="Only process no-blacklist set")
    args = parser.parse_args()

    total_added  = 0
    total_failed = 0

    if not args.no_blacklist_only:
        added, failed = process_set(SET_BLACKLIST, CSV_BLACKLIST, blacklist=True)
        total_added  += added
        total_failed += failed

    if not args.blacklist_only:
        added, failed = process_set(SET_NO_BLACKLIST, CSV_NO_BLACKLIST, blacklist=False)
        total_added  += added
        total_failed += failed

    print(f"\n{'='*60}")
    print(f"DONE. Total added: {total_added} | Total failed: {total_failed}")


if __name__ == '__main__':
    main()