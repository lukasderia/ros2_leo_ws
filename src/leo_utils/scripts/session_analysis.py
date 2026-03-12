#!/usr/bin/env python3
"""
Session Metrics Extractor
Walks a session folder, extracts per-run metrics from each bag,
and appends new rows to a CSV file (skips already-processed runs).

Usage:
    python3 extract_session_metrics.py <session_folder> [--csv output.csv] [--mode rss]

Requirements:
    source /opt/ros/foxy/setup.bash
    source ~/ros2_leo_ws/install/setup.bash

Dependencies: bag_analysis.py must be in the same directory (or on PYTHONPATH).
"""

import sys
import os
import re
import glob
import argparse
import csv
from datetime import datetime

import numpy as np

# ── Reuse all logic from analyze_bag.py ──────────────────────────────────────
# We import the functions directly rather than duplicating them.
# Make sure analyze_bag.py is in the same directory as this script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_analysis import (
    read_topics,
    extract_robot_trajectory,
    map_coverage_over_time,
    extract_rss_cloud,
    extract_velocity,
    extract_goals,
    compute_path_length,
    distance_to_router,
    find_discovery_time,
    parse_router_from_name,
)

# ── CSV schema ────────────────────────────────────────────────────────────────
CSV_COLUMNS = [
    "bag_name",
    "date",
    "mode",
    "stddev",
    "router_x",
    "router_y",
    "robot_start_x",
    "robot_start_y",
    "start_distance",          # euclidean robot-start to router
    "termination_reason",
    "success",
    "duration_s",
    "area_mapped_m2",
    "total_distance_m",
    "avg_speed_ms",
    "min_distance_to_router",
    "final_distance_to_router",
    "mean_approach_rate",      # avg d(distance)/dt — negative = approaching
    "goals_published",
]


# ── Name parsers ──────────────────────────────────────────────────────────────

def parse_robot_start_from_name(bag_name):
    """Extract bx, by (robot start) from bag folder name."""
    match = re.search(r'_bx([-\d.]+)_by([-\d.]+)', bag_name)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None, None


def parse_date_from_name(bag_name):
    """Extract date string like '10Mar_22h41' from bag name."""
    match = re.search(r'(\d+[A-Za-z]+_\d+h\d+)', bag_name)
    return match.group(1) if match else "unknown"


def parse_stddev_from_session(session_folder):
    """
    Extract stddev from session folder name.
    Expects something like: session_rss_2  (last number = stddev)
    """
    name = os.path.basename(session_folder.rstrip('/'))
    match = re.search(r'_(\d+)$', name)
    return int(match.group(1)) if match else -1


def parse_mode_from_session(session_folder, override=None):
    """
    Infer mode from session folder name or use override.
    Folder names expected: session_rss_*, session_yamauchi_*, session_gao_*
    """
    if override:
        return override
    name = os.path.basename(session_folder.rstrip('/'))
    if 'rss' in name.lower():
        return 'rss'
    if 'yamauchi' in name.lower():
        return 'yamauchi'
    if 'gao' in name.lower():
        return 'gao'
    return 'unknown'


def parse_termination_from_result_json(bag_path):
    """Read termination_reason from result.json if present."""
    result_path = os.path.join(bag_path, 'result.json')
    if not os.path.exists(result_path):
        return "unknown"
    import json
    try:
        with open(result_path) as f:
            data = json.load(f)
        return data.get("termination_reason", "unknown")
    except Exception:
        return "unknown"


# ── Per-bag extraction ────────────────────────────────────────────────────────

def extract_metrics(bag_path, router_x, router_y, mode, stddev):
    """
    Extract all per-run scalar metrics from a single bag.
    Returns a dict matching CSV_COLUMNS, or None on failure.
    """
    bag_name = os.path.basename(bag_path.rstrip('/'))
    print(f"  Processing: {bag_name}")

    robot_start_x, robot_start_y = parse_robot_start_from_name(bag_name)
    date_str = parse_date_from_name(bag_name)
    termination = parse_termination_from_result_json(bag_path)

    topics_needed = ['/tf', '/map', '/cmd_vel', '/goal_pose']

    try:
        data = read_topics(bag_path, topics_needed)
    except Exception as e:
        print(f"    ERROR reading bag: {e}")
        return None

    # ── Trajectory ────────────────────────────────────────────────────────────
    trajectory = extract_robot_trajectory(data['/tf'])
    if len(trajectory) < 2:
        print(f"    WARNING: Not enough trajectory points, skipping.")
        return None

    # Normalize time to t=0
    t0 = trajectory[0, 0]
    trajectory[:, 0] -= t0

    # Robot start (first pose in map frame)
    if robot_start_x is None:
        robot_start_x = float(trajectory[0, 1])
        robot_start_y = float(trajectory[0, 2])

    # ── Duration ──────────────────────────────────────────────────────────────
    duration = float(trajectory[-1, 0])

    # ── Area mapped ───────────────────────────────────────────────────────────
    map_times, cov = map_coverage_over_time(data['/map'])
    area_mapped = float(cov[-1]) if len(cov) > 0 else 0.0

    # ── Navigation ────────────────────────────────────────────────────────────
    total_distance = compute_path_length(trajectory)
    avg_speed = total_distance / duration if duration > 0 else 0.0

    # ── Distance to router ────────────────────────────────────────────────────
    dist_times, dist_arr = distance_to_router(trajectory, router_x, router_y)
    min_dist = float(dist_arr.min()) if len(dist_arr) else -1.0
    final_dist = float(dist_arr[-1]) if len(dist_arr) else -1.0

    # Mean approach rate: average of d(distance)/dt
    # Negative value means the robot is on average approaching the router
    if len(dist_arr) > 1:
        dt = np.diff(dist_times)
        dd = np.diff(dist_arr)
        # Avoid division by zero
        valid = dt > 0
        approach_rate = float(np.mean(dd[valid] / dt[valid])) if valid.any() else 0.0
    else:
        approach_rate = 0.0

    # ── Goals ─────────────────────────────────────────────────────────────────
    goals = extract_goals(data['/goal_pose'])
    goals_published = len(goals)

    # ── Derived ───────────────────────────────────────────────────────────────
    start_distance = float(np.sqrt(
        (robot_start_x - router_x)**2 + (robot_start_y - router_y)**2
    ))
    success = "yes" if termination == "router_found" else "no"

    return {
        "bag_name":                bag_name,
        "date":                    date_str,
        "mode":                    mode,
        "stddev":                  stddev,
        "router_x":                router_x,
        "router_y":                router_y,
        "robot_start_x":           round(robot_start_x, 3),
        "robot_start_y":           round(robot_start_y, 3),
        "start_distance":          round(start_distance, 3),
        "termination_reason":      termination,
        "success":                 success,
        "duration_s":              round(duration, 2),
        "area_mapped_m2":          round(area_mapped, 2),
        "total_distance_m":        round(total_distance, 2),
        "avg_speed_ms":            round(avg_speed, 4),
        "min_distance_to_router":  round(min_dist, 3),
        "final_distance_to_router":round(final_dist, 3),
        "mean_approach_rate":      round(approach_rate, 5),
        "goals_published":         goals_published,
    }


# ── CSV helpers ───────────────────────────────────────────────────────────────

def load_existing_bag_names(csv_path):
    """Return set of bag_names already in the CSV."""
    if not os.path.exists(csv_path):
        return set()
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        return {row["bag_name"] for row in reader}


def append_rows(csv_path, rows):
    """Append rows to CSV, creating with header if it doesn't exist."""
    file_exists = os.path.exists(csv_path)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        if not file_exists:
            writer.writeheader()
        writer.writerows(rows)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Extract per-run metrics from a session folder.")
    parser.add_argument("session_folder", help="Path to session folder containing bag subdirectories")
    parser.add_argument("--csv", default=None, help="Output CSV path (default: <session_folder>/metrics.csv)")
    parser.add_argument("--mode", default=None, help="Override mode name (rss/yamauchi/gao)")
    args = parser.parse_args()

    session_folder = args.session_folder.rstrip('/')
    if not os.path.isdir(session_folder):
        print(f"ERROR: Not a directory: {session_folder}")
        sys.exit(1)

    csv_path = args.csv or os.path.join(session_folder, "metrics.csv")
    mode = parse_mode_from_session(session_folder, override=args.mode)
    stddev = parse_stddev_from_session(session_folder)

    print(f"Session:  {session_folder}")
    print(f"Mode:     {mode}")
    print(f"Stddev:   {stddev}")
    print(f"CSV:      {csv_path}")
    print()

    # Find all bag subdirectories (contain a .db3 file)
    bag_dirs = sorted([
        d for d in glob.glob(os.path.join(session_folder, '*/'))
        if glob.glob(os.path.join(d, '*.db3'))
    ])

    if not bag_dirs:
        print("ERROR: No bag directories found.")
        sys.exit(1)

    print(f"Found {len(bag_dirs)} bags.")

    # Load already-processed bags to avoid duplicates
    already_processed = load_existing_bag_names(csv_path)
    print(f"Already in CSV: {len(already_processed)} runs.\n")

    new_rows = []
    skipped = 0
    failed = 0

    for bag_path in bag_dirs:
        bag_name = os.path.basename(bag_path.rstrip('/'))

        if bag_name in already_processed:
            print(f"  Skipping (already in CSV): {bag_name}")
            skipped += 1
            continue

        router_x, router_y = parse_router_from_name(bag_path)
        if router_x is None:
            print(f"  WARNING: Could not parse router position from {bag_name}, skipping.")
            failed += 1
            continue

        row = extract_metrics(bag_path, router_x, router_y, mode, stddev)
        if row is None:
            failed += 1
            continue

        new_rows.append(row)
        print(f"    -> {row['termination_reason']}, {row['duration_s']}s, "
              f"area={row['area_mapped_m2']}m², min_dist={row['min_distance_to_router']}m")

    if new_rows:
        append_rows(csv_path, new_rows)
        print(f"\nAppended {len(new_rows)} new rows to {csv_path}")
    else:
        print("\nNo new rows to add.")

    print(f"Skipped: {skipped} | Failed: {failed} | Added: {len(new_rows)}")


if __name__ == '__main__':
    main()