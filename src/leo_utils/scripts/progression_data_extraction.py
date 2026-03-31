#!/usr/bin/env python3
"""
extract_progression.py  –  Build normalized distance-progression matrices
from ROS2 bags for thesis analysis.

Usage:
    python3 extract_progression.py yamauchi
    python3 extract_progression.py gao
    python3 extract_progression.py rss

Output (saved next to this script):
    <mode>_progression.npy        – (N, RESOLUTION) float array
                                    rows    = runs
                                    columns = time steps (0 to MAX_DURATION)
                                    values  = normalized distance (d / d_initial)
                                    NaN     = run had already terminated
    <mode>_progression_meta.json  – list of dicts, one per row, with bag name,
                                    termination reason, duration, and router pos

X-axis interpretation:
    column index / RESOLUTION * MAX_DURATION = real time in seconds
    e.g. column 500 = 300 seconds into the run

Requirements:
    source /opt/ros/foxy/setup.bash
    source ~/ros2_leo_ws/install/setup.bash
"""

import sys
import os
import json
import glob
import sqlite3
import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("ERROR: rclpy not found. Source ROS2 first:")
    print("  source /opt/ros/foxy/setup.bash")
    print("  source ~/ros2_leo_ws/install/setup.bash")
    sys.exit(1)

# ── Configuration ──────────────────────────────────────────────────────────────

BASE_PATH    = "/media/lukas/VERBATIM HD/Lukas Master"
SESSIONS     = ["Session_no_blacklist", "Session_blacklist"]
RESOLUTION   = 1000       # number of columns in the output matrix
MAX_DURATION = 600.0      # seconds — hardcoded timeout value
SUCCESS_KEY  = "router_found"

MODE_FOLDERS = {
    "yamauchi": ["session_yamauchi"],
    "gao":      ["session_gao"],
    "rss":      ["session_rss_1", "session_rss_2", "session_rss_3", "session_rss_4"],
}


# ── Bag reading ────────────────────────────────────────────────────────────────

def find_db3(bag_path):
    if bag_path.endswith('.db3'):
        return bag_path
    matches = glob.glob(os.path.join(bag_path, '*.db3'))
    if not matches:
        return None
    return matches[0]


def read_tf(bag_path):
    db3 = find_db3(bag_path)
    if db3 is None:
        print(f"    WARNING: no .db3 in {bag_path}")
        return []

    conn = sqlite3.connect(db3)
    cur  = conn.execute("SELECT name, type FROM topics")
    type_map = {row[0]: row[1] for row in cur.fetchall()}

    if '/tf' not in type_map:
        print(f"    WARNING: /tf not found in bag")
        conn.close()
        return []

    cur   = conn.execute("SELECT id FROM topics WHERE name='/tf'")
    tf_id = cur.fetchone()[0]

    try:
        cur  = conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tf_id,)
        )
        rows = cur.fetchall()
    except sqlite3.DatabaseError as e:
        print(f"    WARNING: corrupted bag ({e})")
        conn.close()
        return []

    conn.close()

    msg_type = get_message(type_map['/tf'])
    result = []
    for ts, raw in rows:
        try:
            msg = deserialize_message(bytes(raw), msg_type)
            result.append((ts, msg))
        except Exception:
            pass
    return result


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
            if tf.header.frame_id == 'map'  and tf.child_frame_id == 'odom':
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
    Columns after run termination are NaN.
    """
    try:
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

        # Normalize y by initial distance
        d_norm = d_raw / d_raw[0]

        # Duration of this run in seconds
        duration = t_raw[-1]

        # How many columns this run occupies out of RESOLUTION
        n_cols = int((duration / MAX_DURATION) * RESOLUTION)
        n_cols = min(n_cols, RESOLUTION)  # safety clamp

        # Common time grid for the full MAX_DURATION window
        full_grid = np.linspace(0.0, MAX_DURATION, RESOLUTION)

        # Interpolate only over the portion this run covers
        run_grid = full_grid[:n_cols]
        d_interp = np.interp(run_grid, t_raw, d_norm)

        # Fill the rest with NaN
        row = np.full(RESOLUTION, np.nan)
        row[:n_cols] = d_interp

        return row, duration

    except Exception as e:
        print(f"    WARNING: failed to process bag ({e})")
        return None


# ── Directory walking ──────────────────────────────────────────────────────────

def find_bag_dirs(session_path):
    bag_dirs = []
    for entry in sorted(os.listdir(session_path)):
        candidate = os.path.join(session_path, entry)
        if os.path.isdir(candidate):
            if glob.glob(os.path.join(candidate, '*.db3')):
                bag_dirs.append(candidate)
    return bag_dirs


def find_json(bag_dir):
    candidate = os.path.join(bag_dir, 'result.json')
    if os.path.exists(candidate):
        return candidate
    parent   = os.path.dirname(bag_dir)
    bag_name = os.path.basename(bag_dir)
    candidate = os.path.join(parent, bag_name + '.json')
    if os.path.exists(candidate):
        return candidate
    matches = glob.glob(os.path.join(bag_dir, '*.json'))
    if matches:
        return matches[0]
    return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2 or sys.argv[1] not in MODE_FOLDERS:
        print("Usage: python3 extract_progression.py <mode>")
        print("  mode: yamauchi | gao | rss")
        sys.exit(1)

    mode           = sys.argv[1]
    target_folders = MODE_FOLDERS[mode]

    all_rows = []
    all_meta = []

    for session in SESSIONS:
        session_base = os.path.join(BASE_PATH, session)
        if not os.path.isdir(session_base):
            print(f"WARNING: session folder not found: {session_base}")
            continue

        for folder_name in target_folders:
            folder_path = os.path.join(session_base, folder_name)
            if not os.path.isdir(folder_path):
                print(f"WARNING: folder not found: {folder_path}")
                continue

            bag_dirs = find_bag_dirs(folder_path)
            print(f"\n[{session}/{folder_name}] found {len(bag_dirs)} bag(s)")

            for bag_dir in bag_dirs:
                bag_name = os.path.basename(bag_dir)
                print(f"  Processing: {bag_name}")

                json_path = find_json(bag_dir)
                if json_path is None:
                    print(f"    SKIP: no result.json found")
                    continue

                with open(json_path) as f:
                    result = json.load(f)

                termination = result.get("termination_reason", "unknown")

                router_pos = result.get("router_position")
                if router_pos is None:
                    print(f"    SKIP: no router_position in JSON")
                    continue
                router_x, router_y = float(router_pos[0]), float(router_pos[1])

                outcome = process_bag(bag_dir, router_x, router_y)
                if outcome is None:
                    print(f"    SKIP: processing failed")
                    continue

                row, duration = outcome
                all_rows.append(row)
                all_meta.append({
                    "bag_name":    bag_name,
                    "session":     session,
                    "folder":      folder_name,
                    "termination": termination,
                    "duration_s":  round(duration, 1),
                    "router_x":    router_x,
                    "router_y":    router_y,
                    "path":        bag_dir,
                })
                n_valid = int(np.sum(~np.isnan(row)))
                print(f"    OK  termination={termination:12s}  "
                      f"duration={duration:.1f}s  "
                      f"cols={n_valid}/{RESOLUTION}  "
                      f"d_final={row[n_valid-1]:.3f}")

    if not all_rows:
        print("\nERROR: no runs found. Check paths and JSON keys.")
        sys.exit(1)

    matrix = np.stack(all_rows)

    out_dir   = os.path.dirname(os.path.abspath(__file__))
    npy_path  = os.path.join(out_dir, f"{mode}_progression.npy")
    meta_path = os.path.join(out_dir, f"{mode}_progression_meta.json")

    np.save(npy_path, matrix)
    with open(meta_path, 'w') as f:
        json.dump(all_meta, f, indent=2)

    # Summary
    print(f"\n── Summary ──────────────────────────────")
    terminations = [m["termination"] for m in all_meta]
    for t in sorted(set(terminations)):
        print(f"  {t}: {terminations.count(t)}")
    print(f"\nTotal runs: {len(all_rows)}")
    print(f"Matrix shape: {matrix.shape}  (runs x time_steps)")
    print(f"Saved: {npy_path}")
    print(f"Saved: {meta_path}")


if __name__ == '__main__':
    main()