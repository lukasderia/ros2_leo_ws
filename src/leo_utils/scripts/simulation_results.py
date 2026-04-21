#!/usr/bin/env python3
"""
extract_progression.py  –  Build normalized distance-progression matrices
from ROS2 bags for a given session folder.

Usage:
    python3 extract_progression.py <session_folder_path>

Example:
    python3 extract_progression.py "/Volumes/KINGSTON/Lukas Master/Session_final/session_yamauchi"

Output (saved next to this script):
    <folder_name>_progression.npy       – (N, RESOLUTION) float array
                                          rows    = runs (flips excluded)
                                          columns = time steps (0 to MAX_DURATION)
                                          values  = normalized distance (d / d_initial)
    <folder_name>_progression_meta.json – list of dicts, one per row

Requirements:
    pip install rosbags numpy
"""

import sys
import os
import json
import glob
import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    print("ERROR: rosbags not found. Install it with:")
    print("  pip install rosbags")
    sys.exit(1)

# ── Configuration ──────────────────────────────────────────────────────────────

RESOLUTION   = 300        # number of time steps in output matrix
MAX_DURATION = 600.0      # seconds
SKIP_REASONS = {"flip"}   # termination reasons to skip

# ── Bag reading ────────────────────────────────────────────────────────────────

def read_tf(bag_path):
    """Read all /tf messages from a bag. Returns list of (timestamp_ns, msg)."""
    typestore = get_typestore(Stores.ROS2_FOXY)
    results = []

    try:
        with Reader(bag_path) as reader:
            connections = [c for c in reader.connections if c.topic == '/tf']
            if not connections:
                print(f"    WARNING: /tf not found in bag")
                return []

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                results.append((timestamp, msg))
    except Exception as e:
        print(f"    WARNING: failed to read bag ({e})")
        return []

    return results


# ── Transform helpers ──────────────────────────────────────────────────────────

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
    """Returns (N, 3) array of [t_sec, x, y] in map frame."""
    map_odom  = []
    odom_base = []

    for ts, msg in tf_messages:
        for tf in msg.transforms:
            p = tf.transform.translation
            r = tf.transform.rotation
            entry = (ts, p.x, p.y, p.z, r.x, r.y, r.z, r.w)
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                map_odom.append(entry)
            elif tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                odom_base.append(entry)

    if not map_odom or not odom_base:
        return np.array([]).reshape(0, 3)

    map_odom.sort(key=lambda x: x[0])
    odom_base.sort(key=lambda x: x[0])
    mo_ts = np.array([e[0] for e in map_odom])

    trajectory = []
    for ob in odom_base:
        ts  = ob[0]
        idx = min(np.searchsorted(mo_ts, ts), len(map_odom) - 1)
        mo  = map_odom[idx]
        t_out, _ = compose_transforms(
            [mo[1], mo[2], mo[3]], [mo[4], mo[5], mo[6], mo[7]],
            [ob[1], ob[2], ob[3]], [ob[4], ob[5], ob[6], ob[7]]
        )
        trajectory.append((ts / 1e9, t_out[0], t_out[1]))

    trajectory.sort(key=lambda x: x[0])
    return np.array(trajectory)


# ── Core: process one bag ──────────────────────────────────────────────────────

def process_bag(bag_path, router_x, router_y):
    """
    Returns (row, duration) where row is a 1D numpy array of length RESOLUTION.

    X-axis: real time 0 → MAX_DURATION, divided into RESOLUTION steps.
    Y-axis: normalized distance d(t) / d_initial.
    After run termination: held constant at final value.
    """
    tf_messages = read_tf(bag_path)
    if not tf_messages:
        return None

    trajectory = extract_robot_trajectory(tf_messages)
    if len(trajectory) < 2:
        print(f"    WARNING: too few trajectory points ({len(trajectory)})")
        return None

    # Shift time to start at 0
    t_raw = trajectory[:, 0] - trajectory[0, 0]
    d_raw = np.sqrt(
        (trajectory[:, 1] - router_x)**2 +
        (trajectory[:, 2] - router_y)**2
    )

    # Normalize by initial distance
    d_norm = d_raw / d_raw[0]

    duration = t_raw[-1]

    # How many columns this run occupies
    n_cols = int((duration / MAX_DURATION) * RESOLUTION)
    n_cols = min(n_cols, RESOLUTION)

    full_grid = np.linspace(0.0, MAX_DURATION, RESOLUTION)
    run_grid  = full_grid[:n_cols]
    d_interp  = np.interp(run_grid, t_raw, d_norm)

    # Hold final value constant for remainder
    row = np.full(RESOLUTION, np.nan)
    row[:n_cols] = d_interp
    row[n_cols:] = d_interp[-1]

    return row, duration


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    session_path = sys.argv[1].rstrip('/')
    folder_name  = os.path.basename(session_path)

    if not os.path.isdir(session_path):
        print(f"ERROR: folder not found: {session_path}")
        sys.exit(1)

    out_dir   = os.path.dirname(os.path.abspath(__file__))
    npy_path  = os.path.join(out_dir, f"{folder_name}_progression.npy")
    meta_path = os.path.join(out_dir, f"{folder_name}_progression_meta.json")

    # ── Check if already extracted ─────────────────────────────────────────────
    if os.path.exists(npy_path) and os.path.exists(meta_path):
        print(f"Cache found for {folder_name}, loading from disk.")
        print(f"  (Delete {npy_path} to force re-extraction.)")
        matrix = np.load(npy_path)
        with open(meta_path) as f:
            all_meta = json.load(f)

    else:
        # ── Extract from bags ──────────────────────────────────────────────────
        bag_dirs = sorted([
            os.path.join(session_path, d)
            for d in os.listdir(session_path)
            if os.path.isdir(os.path.join(session_path, d))
            and glob.glob(os.path.join(session_path, d, '*.db3'))
        ])

        print(f"Found {len(bag_dirs)} bag(s) in {folder_name}")

        all_rows = []
        all_meta = []
        skipped  = 0

        for bag_dir in bag_dirs:
            bag_name  = os.path.basename(bag_dir)
            json_path = os.path.join(bag_dir, 'result.json')

            if not os.path.exists(json_path):
                print(f"  SKIP {bag_name}: no result.json")
                skipped += 1
                continue

            with open(json_path) as f:
                result = json.load(f)

            termination = result.get("termination_reason", "unknown")

            if termination in SKIP_REASONS:
                print(f"  SKIP {bag_name}: {termination}")
                skipped += 1
                continue

            router_pos = result.get("router_position")
            if router_pos is None:
                print(f"  SKIP {bag_name}: no router_position in result.json")
                skipped += 1
                continue

            router_x, router_y = float(router_pos[0]), float(router_pos[1])

            print(f"  Processing {bag_name}  [{termination}]")

            outcome = process_bag(bag_dir, router_x, router_y)
            if outcome is None:
                print(f"    SKIP: processing failed  <-- {bag_name}")
                skipped += 1
                continue

            row, duration = outcome
            all_rows.append(row)
            all_meta.append({
                "bag_name":    bag_name,
                "termination": termination,
                "duration_s":  round(duration, 1),
                "router_x":    router_x,
                "router_y":    router_y,
                "path":        bag_dir,
            })
            print(f"    OK  duration={duration:.1f}s  d_final={row[~np.isnan(row)][-1]:.3f}")

        if not all_rows:
            print("\nERROR: no valid runs found.")
            sys.exit(1)

        matrix = np.stack(all_rows)

        np.save(npy_path, matrix)
        with open(meta_path, 'w') as f:
            json.dump(all_meta, f, indent=2)

        print(f"\n── Extraction summary ────────────────────")
        terminations = [m["termination"] for m in all_meta]
        for t in sorted(set(terminations)):
            print(f"  {t}: {terminations.count(t)}")
        print(f"  skipped: {skipped}")
        print(f"\nTotal runs saved: {len(all_rows)}")
        print(f"Matrix shape: {matrix.shape}  (runs x time_steps)")
        print(f"Saved: {npy_path}")
        print(f"Saved: {meta_path}")

    # ── Placeholder for future processing steps ────────────────────────────────
    # matrix and all_meta are available here regardless of whether
    # they were freshly extracted or loaded from cache.
    print(f"\nReady: {matrix.shape[0]} runs, {matrix.shape[1]} time steps")


if __name__ == '__main__':
    main()