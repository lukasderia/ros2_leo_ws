#!/usr/bin/env python3
"""
simulation_results.py  –  Extract and plot progression matrices for all modes.

Usage:
    python3 simulation_results.py <session_root_path>

Example:
    python3 simulation_results.py "/Volumes/KINGSTON/Lukas Master/Session_final"

For each mode the script will:
    1. Check if a cached .npy already exists — if so, skip extraction.
    2. Otherwise walk the session folder(s), skip flips, and build the matrix.
    3. Save <mode>_progression.npy and <mode>_progression_meta.json next to the script.

After all modes are processed it produces one plot per mode showing all runs,
saved to a progression_plots/ folder next to the script.

Requirements:
    pip install rosbags numpy matplotlib
"""

import sys
import os
import json
import glob
import numpy as np
import matplotlib
matplotlib.use('Agg')  # non-interactive backend, saves to file
import matplotlib.pyplot as plt

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    print("ERROR: rosbags not found.  pip install rosbags")
    sys.exit(1)

# ── Configuration ──────────────────────────────────────────────────────────────

RESOLUTION   = 600
MAX_DURATION = 600.0
SKIP_REASONS = {"flip"}

MODES = {
    "yamauchi": {"folders": ["session_yamauchi"], "cache_prefix": "yamauchi"},
    "gao":      {"folders": ["session_gao"],      "cache_prefix": "gao"},
    "rss":      {"folders": ["session_rss_1", "session_rss_2", "session_rss_3", "session_rss_4"],
                 "cache_prefix": "rss"},
}

MODE_COLORS = {
    "yamauchi": "steelblue",
    "gao":      "darkorange",
    "rss":      "seagreen",
}

# ── Bag reading ────────────────────────────────────────────────────────────────

def read_tf(bag_path):
    typestore = get_typestore(Stores.ROS2_FOXY)
    results   = []
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
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
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
            [ob[1], ob[2], ob[3]], [ob[4], ob[5], ob[6], ob[7]],
        )
        trajectory.append((ts / 1e9, t_out[0], t_out[1]))

    trajectory.sort(key=lambda x: x[0])
    return np.array(trajectory)


# ── Core: process one bag ──────────────────────────────────────────────────────

def process_bag(bag_path, router_x, router_y):
    tf_messages = read_tf(bag_path)
    if not tf_messages:
        return None

    trajectory = extract_robot_trajectory(tf_messages)
    if len(trajectory) < 2:
        print(f"    WARNING: too few trajectory points ({len(trajectory)})")
        return None

    t_raw = trajectory[:, 0] - trajectory[0, 0]
    d_raw = np.sqrt(
        (trajectory[:, 1] - router_x)**2 +
        (trajectory[:, 2] - router_y)**2
    )
    d_norm = d_raw / d_raw[0]
    duration = t_raw[-1]

    n_cols = min(int((duration / MAX_DURATION) * RESOLUTION), RESOLUTION)

    full_grid = np.linspace(0.0, MAX_DURATION, RESOLUTION)
    d_interp  = np.interp(full_grid[:n_cols], t_raw, d_norm)

    row          = np.full(RESOLUTION, np.nan)
    row[:n_cols] = d_interp
    row[n_cols:] = d_interp[-1]   # hold final value for remainder

    return row, duration


# ── Extraction: one mode ───────────────────────────────────────────────────────

def extract_mode(session_root, mode, folders, out_dir):
    """
    Extract or load the progression matrix for one mode.
    Returns (matrix, meta) where matrix is (N, RESOLUTION).
    """
    npy_path  = os.path.join(out_dir, f"{mode}_progression.npy")
    meta_path = os.path.join(out_dir, f"{mode}_progression_meta.json")

    # For modes with multiple folders (e.g. rss), check for per-folder cache files
    per_folder_npys  = [os.path.join(out_dir, f"{folder.replace('session_', '')}_progression.npy") for folder in folders]
    per_folder_metas = [os.path.join(out_dir, f"{folder.replace('session_', '')}_progression_meta.json") for folder in folders]

    if all(os.path.exists(p) for p in per_folder_npys + per_folder_metas):
        print(f"  [{mode}] Per-folder cache found, loading and combining.")
        matrix = np.vstack([np.load(p) for p in per_folder_npys])
        meta   = []
        for mp in per_folder_metas:
            with open(mp) as f:
                meta.extend(json.load(f))
        print(f"           Loaded {matrix.shape[0]} runs.")
        return matrix, meta

    elif os.path.exists(npy_path) and os.path.exists(meta_path):
        print(f"  [{mode}] Cache found, loading from disk.")
        print(f"           (Delete {npy_path} to force re-extraction.)")
        matrix = np.load(npy_path)
        with open(meta_path) as f:
            meta = json.load(f)
        print(f"           Loaded {matrix.shape[0]} runs.")
        return matrix, meta

    print(f"  [{mode}] Extracting from bags ...")

    all_rows = []
    all_meta = []
    skipped  = 0

    for folder_name in folders:
        folder_path = os.path.join(session_root, folder_name)
        if not os.path.isdir(folder_path):
            print(f"    WARNING: folder not found: {folder_path}")
            continue

        bag_dirs = sorted([
            os.path.join(folder_path, d)
            for d in os.listdir(folder_path)
            if os.path.isdir(os.path.join(folder_path, d))
            and glob.glob(os.path.join(folder_path, d, '*.db3'))
        ])

        print(f"    {folder_name}: {len(bag_dirs)} bag(s) found")

        for bag_dir in bag_dirs:
            bag_name  = os.path.basename(bag_dir)
            json_path = os.path.join(bag_dir, 'result.json')

            if not os.path.exists(json_path):
                print(f"      SKIP {bag_name}: no result.json")
                skipped += 1
                continue

            with open(json_path) as f:
                result = json.load(f)

            termination = result.get("termination_reason", "unknown")

            if termination in SKIP_REASONS:
                skipped += 1
                continue

            router_pos = result.get("router_position")
            if router_pos is None:
                print(f"      SKIP {bag_name}: no router_position")
                skipped += 1
                continue

            router_x, router_y = float(router_pos[0]), float(router_pos[1])

            outcome = process_bag(bag_dir, router_x, router_y)
            if outcome is None:
                print(f"      SKIP {bag_name}: processing failed")
                skipped += 1
                continue

            row, duration = outcome
            all_rows.append(row)
            all_meta.append({
                "bag_name":    bag_name,
                "folder":      folder_name,
                "termination": termination,
                "duration_s":  round(duration, 1),
                "router_x":    router_x,
                "router_y":    router_y,
            })

    if not all_rows:
        print(f"  [{mode}] ERROR: no valid runs found.")
        return None, None

    matrix = np.stack(all_rows)
    np.save(npy_path, matrix)
    with open(meta_path, 'w') as f:
        json.dump(all_meta, f, indent=2)

    terminations = [m["termination"] for m in all_meta]
    print(f"  [{mode}] Done: {len(all_rows)} runs saved  "
          f"(skipped {skipped},  "
          + "  ".join(f"{t}: {terminations.count(t)}" for t in sorted(set(terminations))) + ")")

    return matrix, all_meta


# ── Plotting ───────────────────────────────────────────────────────────────────

def plot_mode(mode, matrix, meta, plot_dir):
    x     = np.linspace(0, MAX_DURATION, RESOLUTION)
    color = MODE_COLORS.get(mode, "grey")

    fig, ax = plt.subplots(figsize=(12, 6))

    for row in matrix:
        ax.plot(x, row, color=color, alpha=0.2, linewidth=0.7)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Normalized distance to router  (d / d₀)')
    ax.set_title(f'{mode}  —  all runs  ({matrix.shape[0]} total)')
    ax.set_xlim(0, MAX_DURATION)
    ax.set_ylim(0, 1.2)
    ax.set_xticks([0, 120, 240, 360, 480, 600])
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(plot_dir, f"{mode}_all_runs.png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"  Saved: {out_path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    session_root = sys.argv[1].rstrip('/')

    if not os.path.isdir(session_root):
        print(f"ERROR: folder not found: {session_root}")
        sys.exit(1)

    out_dir  = os.path.dirname(os.path.abspath(__file__))
    plot_dir = os.path.join(out_dir, "progression_plots")

    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)
        print(f"Created plot folder: {plot_dir}")

    # ── Extract all modes ──────────────────────────────────────────────────────
    print("\n── Extraction ────────────────────────────────────────────────────────")
    mode_data = {}
    for mode, config in MODES.items():
        matrix, meta = extract_mode(session_root, mode, config["folders"], out_dir)
        if matrix is not None:
            mode_data[mode] = (matrix, meta)

    # ── Plot all modes ─────────────────────────────────────────────────────────
    print("\n── Plotting ──────────────────────────────────────────────────────────")
    for mode, (matrix, meta) in mode_data.items():
        plot_mode(mode, matrix, meta, plot_dir)

    print("\nDone.")


if __name__ == '__main__':
    main()