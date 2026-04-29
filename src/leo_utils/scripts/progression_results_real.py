#!/usr/bin/env python3
"""
progression_real.py  —  Extract and plot real robot run progressions.

Walks bags_real/session_yamauchi, session_gao, session_rss.
Reads info.json per run (post_fix only).
Plots all runs on a single figure, colored by mode, no aggregation.

Usage:
    python3 progression_real.py <bags_real_path>
"""

import sys
import os
import json
import glob
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    print("ERROR: rosbags not found.  pip install rosbags")
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════════════════
# ── Config ────────────────────────────────────════════════════════════════════
# ══════════════════════════════════════════════════════════════════════════════

MAX_DURATION = 600.0
RESOLUTION   = 600   # 1 column per second

SHOW_INDIVIDUAL_RUNS = True

SESSIONS = {
    "yamauchi": "session_yamauchi",
    "gao":      "session_gao",
    "rss":      "session_rss",
}

MODE_COLORS = {
    "yamauchi": "#C0392B",
    "gao":      "#E67E22",
    "rss":      "#3266AD",
}

MODE_LABELS = {
    "yamauchi": "Yamauchi",
    "gao":      "Gao",
    "rss":      "RSS",
}

# ══════════════════════════════════════════════════════════════════════════════
# ── TF extraction (unchanged from simulation script) ─────────────────════════
# ══════════════════════════════════════════════════════════════════════════════

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


def read_tf(bag_path):
    typestore = get_typestore(Stores.ROS2_FOXY)
    results   = []
    try:
        with Reader(bag_path) as reader:
            connections = [c for c in reader.connections if c.topic == '/tf']
            if not connections:
                return []
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                results.append((timestamp, msg))
    except Exception as e:
        print(f"    WARNING: failed to read bag ({e})")
        return []
    return results


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


def process_bag(bag_path, router_x, router_y):
    tf_messages = read_tf(bag_path)
    if not tf_messages:
        return None

    trajectory = extract_robot_trajectory(tf_messages)
    if len(trajectory) < 2:
        return None

    t_raw    = trajectory[:, 0] - trajectory[0, 0]
    d_raw    = np.sqrt((trajectory[:, 1] - router_x)**2 +
                       (trajectory[:, 2] - router_y)**2)
    d_norm   = d_raw / d_raw[0]
    duration = t_raw[-1]

    n_cols    = min(int((duration / MAX_DURATION) * RESOLUTION), RESOLUTION)
    full_grid = np.linspace(0.0, MAX_DURATION, RESOLUTION)
    d_interp  = np.interp(full_grid[:n_cols], t_raw, d_norm)

    row          = np.full(RESOLUTION, np.nan)
    row[:n_cols] = d_interp
    row[n_cols:] = d_interp[-1]   # forward-fill final value

    return row, duration

# ══════════════════════════════════════════════════════════════════════════════
# ── Extraction ────────────────────────────────────────────────────────════════
# ══════════════════════════════════════════════════════════════════════════════

def extract_all(bags_root):
    """
    Walk each session folder, read info.json, extract trajectory.
    Returns dict: mode -> list of dicts with keys: row, duration, termination, combination, bag_name
    """
    data = {mode: [] for mode in SESSIONS}

    for mode, session_folder in SESSIONS.items():
        folder_path = os.path.join(bags_root, session_folder)
        if not os.path.isdir(folder_path):
            print(f"  WARNING: folder not found: {folder_path}")
            continue

        run_dirs = sorted([
            os.path.join(folder_path, d)
            for d in os.listdir(folder_path)
            if os.path.isdir(os.path.join(folder_path, d))
            and glob.glob(os.path.join(folder_path, d, '*.db3'))
        ])
        print(f"  [{mode}] {len(run_dirs)} run(s) found")

        for run_dir in run_dirs:
            bag_name  = os.path.basename(run_dir)
            info_path = os.path.join(run_dir, 'info.json')

            if not os.path.exists(info_path):
                print(f"    SKIP {bag_name}: no info.json")
                continue

            with open(info_path) as f:
                info = json.load(f)

            if info.get("nav2") != "post_fix":
                print(f"    SKIP {bag_name}: nav2={info.get('nav2')} (not post_fix)")
                continue

            router_x = float(info["router_x"])
            router_y = float(info["router_y"])
            termination = info.get("termination_reason", "unknown")
            combination = info.get("combination", "?")

            outcome = process_bag(run_dir, router_x, router_y)
            if outcome is None:
                print(f"    SKIP {bag_name}: bag extraction failed")
                continue

            row, duration = outcome
            data[mode].append({
                "bag_name":    bag_name,
                "row":         row,
                "duration":    round(duration, 1),
                "termination": termination,
                "combination": combination,
                "router_x":    router_x,
                "router_y":    router_y,
            })
            print(f"    OK   {bag_name}  combo={combination}  "
                  f"duration={duration:.0f}s  termination={termination}")

    return data

# ══════════════════════════════════════════════════════════════════════════════
# ── Plotting ──────────────────────────────────────────────────────────════════
# ══════════════════════════════════════════════════════════════════════════════

def plot_all_modes(data, plot_dir):
    """Single figure, all modes, individual lines + mean colored by mode."""
    x   = np.linspace(0, MAX_DURATION, RESOLUTION)
    fig, ax = plt.subplots(figsize=(10, 5))

    for mode, runs in data.items():
        if not runs:
            continue
        color = MODE_COLORS[mode]
        matrix = np.stack([r["row"] for r in runs])
        mean_curve = np.nanmean(matrix, axis=0)
        std_curve = np.nanstd(matrix, axis=0)
        ax.fill_between(x, mean_curve - std_curve, mean_curve + std_curve, color=color, alpha=0.15, linewidth=0)
        n = len(runs)
        if SHOW_INDIVIDUAL_RUNS:
            for run in runs:
                ax.plot(x, run["row"], color=color, alpha=0.3, linewidth=0.9)
        ax.plot(x, mean_curve, color=color, alpha=1.0, linewidth=2.5,
                label=f'{MODE_LABELS[mode]}  (mean, n={n})')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Normalised distance to router  (d / d₀)')
    ax.set_title('Physical runs — normalised progression by mode (post-fix)')
    ax.set_xlim(0, MAX_DURATION)
    ax.set_ylim(0, 1.2)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', frameon=False, fontsize=10)
    plt.tight_layout()

    out_path = os.path.join(plot_dir, 'real_progression_all_modes.pdf')
    plt.savefig(out_path, format='pdf')
    plt.close()
    print(f"  Saved: {out_path}")

def plot_per_mode(data, plot_dir):
    """3x1 subplot, one panel per mode, lines labeled by combination."""
    x     = np.linspace(0, MAX_DURATION, RESOLUTION)
    modes = [m for m in SESSIONS if data.get(m)]

    fig, axes = plt.subplots(len(modes), 1, figsize=(10, 12), sharex=True, sharey=True)
    if len(modes) == 1:
        axes = [axes]

    for ax, mode in zip(axes, modes):
        runs  = data[mode]
        color = MODE_COLORS[mode]

        matrix     = np.stack([r["row"] for r in runs])
        mean_curve = np.nanmean(matrix, axis=0)

        if SHOW_INDIVIDUAL_RUNS:
            for run in runs:
                label = f"combo {run['combination']}  ({run['termination']},  {run['duration']:.0f}s)"
                ax.plot(x, run["row"], color=color, alpha=0.4, linewidth=1.0, label=label)
        ax.plot(x, mean_curve, color=color, linewidth=2.5, label=f'mean (n={len(runs)})')

        ax.set_xlim(0, MAX_DURATION)
        ax.set_ylim(0, 1.25)
        ax.set_ylabel('Normalised distance  (d / d₀)')
        ax.grid(True, alpha=0.3)


    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()

    out_path = os.path.join(plot_dir, 'real_progression_per_mode.pdf')
    plt.savefig(out_path, format='pdf')
    plt.close()
    print(f"  Saved: {out_path}")


# ══════════════════════════════════════════════════════════════════════════════
# ── Main ──────────────────────────────────────────────────────────────════════
# ══════════════════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    bags_root = sys.argv[1].rstrip('/')
    if not os.path.isdir(bags_root):
        print(f"ERROR: folder not found: {bags_root}")
        sys.exit(1)

    plot_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "progression_plots_real")
    os.makedirs(plot_dir, exist_ok=True)

    print("\n── Extraction ───────────────────────────────────────────────────────────")
    data = extract_all(bags_root)

    total = sum(len(v) for v in data.values())
    print(f"\n  Total runs extracted: {total}")
    for mode, runs in data.items():
        print(f"    {mode}: {len(runs)} run(s)")

    print("\n── Plotting ─────────────────────────────────────────────────────────────")
    plot_all_modes(data, plot_dir)
    plot_per_mode(data, plot_dir)

    print("\nDone.")


if __name__ == '__main__':
    main()