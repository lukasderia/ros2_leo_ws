#!/usr/bin/env python3
"""
progression_sampling.py  —  Extract, cache, and plot progression matrices.

Part 1: Extraction
    Walks the session folder, reads bags, builds per-mode .npy cache files.
    Skips any bag ending in a flip. Caches are saved next to the script.
    Re-run is safe — existing caches are loaded from disk, not re-extracted.
    Delete a .npy file to force re-extraction for that mode.

Part 2: Plotting
    Produces per-mode plots:
      - All runs
      - N seeded plots (max MAX_RUNS per combination, random sampling)
      - Grand mean summary across all seeds
    Also produces per-RSS-variant plots and a 2x2 summary.

Usage:
    python3 progression_sampling.py <session_root_path>
"""

import sys
import unicodedata
import os
import re
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
# ── Config ────────────────────────────────────────────────────────────────────
# ══════════════════════════════════════════════════════════════════════════════

RESOLUTION   = 600
MAX_DURATION = 600.0
MAX_RUNS     = 3
SEEDS        = [42]#, 7, 123, 999, 256, 512, 1337, 2024, 888, 314]
SKIP_REASONS = {"flip"}

# Extraction mode config: name -> list of session subfolders
EXTRACT_MODES = {
    "yamauchi": ["session_yamauchi"],
    "gao":      ["session_gao"],
    "rss_1":    ["session_rss_1"],
    "rss_2":    ["session_rss_2"],
    "rss_3":    ["session_rss_3"],
    "rss_4":    ["session_rss_4"],
}

# Combined RSS (loaded by merging rss_1..4 cache files)
PLOT_MODES = ["yamauchi", "gao", "rss"]

# ── Plot flags ────────────────────────────────────────────────────────────────
SHOW_INDIVIDUAL_RUNS = False   # faint lines for each individual run
SHOW_MEAN            = True   # thick mean curve
SHOW_STD_BAND        = True   # shaded std band around mean
STD_MULTIPLIER       = 1.0    # width of std band (e.g. 0.5, 1.0, 2.0)

# ── Output flags ──────────────────────────────────────────────────────────────
OUTPUT_ALL_RUNS     = True   # one plot per mode with all runs
OUTPUT_SEEDED       = True   # one plot per mode per seed
OUTPUT_SUMMARY      = True   # one grand mean summary plot per mode
OUTPUT_RSS_VARIANTS = True   # individual plots per RSS variant + 2x2 summary

# ── Colors ────────────────────────────────────────────────────────────────────
MODE_COLORS = {
    "yamauchi": "#C0392B",
    "gao":      "#E67E22",
    "rss":      "#3266AD",
    "rss_1":    "#1B4F8A",
    "rss_2":    "#3266AD",
    "rss_3":    "#6AA3D9",
    "rss_4":    "#9DC0E8",
}

# ══════════════════════════════════════════════════════════════════════════════
# ── Part 1: Extraction ────────────────────────────────────────────────────────
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
    n_cols   = min(int((duration / MAX_DURATION) * RESOLUTION), RESOLUTION)

    full_grid = np.linspace(0.0, MAX_DURATION, RESOLUTION)
    d_interp  = np.interp(full_grid[:n_cols], t_raw, d_norm)

    row          = np.full(RESOLUTION, np.nan)
    row[:n_cols] = d_interp
    row[n_cols:] = d_interp[-1]

    return row, duration


def extract_mode(session_root, mode, folders, out_dir):
    npy_path  = os.path.join(out_dir, f"{mode}_progression.npy")
    meta_path = os.path.join(out_dir, f"{mode}_progression_meta.json")

    if os.path.exists(npy_path) and os.path.exists(meta_path):
        print(f"  [{mode}] Cache found, loading from disk.")
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
                skipped += 1
                continue

            router_x, router_y = float(router_pos[0]), float(router_pos[1])
            outcome = process_bag(bag_dir, router_x, router_y)
            if outcome is None:
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
    print(f"  [{mode}] Done: {len(all_rows)} runs saved (skipped {skipped}, "
          + "  ".join(f"{t}: {terminations.count(t)}" for t in sorted(set(terminations))) + ")")

    return matrix, all_meta


# ══════════════════════════════════════════════════════════════════════════════
# ── Part 2: Loading ───────────────────────────────────────────────────────────
# ══════════════════════════════════════════════════════════════════════════════

def load_mode(out_dir, mode):
    """Load matrix and meta. For combined 'rss', merges rss_1..4 caches."""
    if mode == "rss":
        npy_path  = os.path.join(out_dir, "rss_4_progression.npy")
        meta_path = os.path.join(out_dir, "rss_4_progression_meta.json")
        if os.path.exists(npy_path) and os.path.exists(meta_path):
            matrix = np.load(npy_path)
            with open(meta_path) as f:
                meta = json.load(f)
            print(f"  [rss] Loaded {matrix.shape[0]} runs from rss_4 cache.")
            return matrix, meta
        print(f"  [rss] Cache not found.")
        return None, None

    npy_path  = os.path.join(out_dir, f"{mode}_progression.npy")
    meta_path = os.path.join(out_dir, f"{mode}_progression_meta.json")
    if not os.path.exists(npy_path) or not os.path.exists(meta_path):
        print(f"  [{mode}] Cache not found.")
        return None, None
    matrix = np.load(npy_path)
    with open(meta_path) as f:
        meta = json.load(f)
    print(f"  [{mode}] Loaded {matrix.shape[0]} runs.")
    return matrix, meta


# ══════════════════════════════════════════════════════════════════════════════
# ── Part 3: Sampling ──────────────────────────────────────────────────────────
# ══════════════════════════════════════════════════════════════════════════════

def parse_combo_key(bag_name):
    rx = re.search(r'rx(-?[0-9.]+)', bag_name)
    ry = re.search(r'ry(-?[0-9.]+)', bag_name)
    bx = re.search(r'bx(-?[0-9.]+)', bag_name)
    by = re.search(r'by(-?[0-9.]+)', bag_name)
    if rx and ry and bx and by:
        return (float(rx.group(1)), float(ry.group(1)),
                float(bx.group(1)), float(by.group(1)))
    return (None, None, None, None)


def sample_indices(meta, seed):
    rng    = np.random.default_rng(seed)
    combos = {}
    for i, m in enumerate(meta):
        key = parse_combo_key(m["bag_name"])
        if key == (None, None, None, None):
            key = (m["router_x"], m["router_y"])
        combos.setdefault(key, []).append(i)

    print(f"      Combinations found: {len(combos)}")
    selected = []
    for key, indices in sorted(combos.items()):
        if len(indices) <= MAX_RUNS:
            selected.extend(indices)
        else:
            chosen = rng.choice(indices, size=MAX_RUNS, replace=False)
            selected.extend(chosen.tolist())
    return selected

def first_indices(meta):
    combos = {}
    for i, m in enumerate(meta):
        key = parse_combo_key(m["bag_name"])
        if key == (None, None, None, None):
            key = (m["router_x"], m["router_y"])
        combos.setdefault(key, []).append(i)

    print(f"      Combinations found: {len(combos)}")
    selected = []
    for key, indices in sorted(combos.items()):
        selected.extend(indices[:MAX_RUNS])
    return selected


# ══════════════════════════════════════════════════════════════════════════════
# ── Part 4: Plotting ──────────────────────────────────────────────────────────
# ══════════════════════════════════════════════════════════════════════════════

def compute_stats(matrix, indices):
    """
    Compute summary statistics for a set of run indices.
    Returns a dict with mean/std at termination, peak std, and AUC.
    """
    subset     = matrix[indices]
    mean_curve = np.nanmean(subset, axis=0)
    std_curve  = np.nanstd(subset, axis=0)

    mean_at_end  = mean_curve[-1]
    std_at_end   = std_curve[-1]
    peak_std     = np.nanmax(std_curve)
    peak_std_t   = np.linspace(0, MAX_DURATION, RESOLUTION)[np.nanargmax(std_curve)]
    auc          = np.trapz(mean_curve, dx=MAX_DURATION / RESOLUTION) / MAX_DURATION

    return {
        "n":            len(indices),
        "mean_at_end":  round(mean_at_end, 3),
        "std_at_end":   round(std_at_end, 3),
        "peak_std":     round(peak_std, 3),
        "peak_std_t":   round(peak_std_t, 0),
        "auc":          round(auc, 3),
    }


def print_stats(mode, stats):
    print(f"  [{mode}]  n={stats['n']}  "
          f"mean@end={stats['mean_at_end']}  "
          f"std@end={stats['std_at_end']}  "
          f"peak_std={stats['peak_std']} @ t={stats['peak_std_t']}s  "
          f"AUC={stats['auc']}")


def print_stats_table(mode_stats_list):
    """Print a formatted table of stats for a list of (mode, stats) tuples."""
    header = f"{'Mode':<12} {'n':>6} {'mean@end':>10} {'std@end':>9} {'peak_std':>10} {'peak_t(s)':>10} {'AUC':>7}"
    sep = chr(10) + unicodedata.lookup('BOX DRAWINGS LIGHT HORIZONTAL') * len(header)
    print(chr(10) + '─' * len(header))
    print(header)
    print('─' * len(header))
    for mode, s in mode_stats_list:
        print(f"{mode:<12} {s['n']:>6} {s['mean_at_end']:>10.3f} "
              f"{s['std_at_end']:>9.3f} {s['peak_std']:>10.3f} "
              f"{s['peak_std_t']:>10.0f} {s['auc']:>7.3f}")
    print('─' * len(header))





def plot_runs(mode, matrix, indices, title, out_path):
    x     = np.linspace(0, MAX_DURATION, RESOLUTION)
    color = MODE_COLORS.get(mode, "grey")
    n     = len(indices)

    subset     = matrix[indices]
    mean_curve = np.nanmean(subset, axis=0)
    std_curve  = np.nanstd(subset, axis=0)

    fig, ax = plt.subplots(figsize=(10, 5))

    if SHOW_INDIVIDUAL_RUNS:
        for i in indices:
            ax.plot(x, matrix[i], color=color, alpha=0.15, linewidth=0.7)
    if SHOW_STD_BAND:
        ax.fill_between(x,
                        mean_curve - STD_MULTIPLIER * std_curve,
                        mean_curve + STD_MULTIPLIER * std_curve,
                        color=color, alpha=0.2, linewidth=0)
    if SHOW_MEAN:
        ax.plot(x, mean_curve, color=color, alpha=1.0, linewidth=2.5)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Normalised distance to router  (d / d₀)')

    ax.set_xlim(0, MAX_DURATION)
    ax.set_ylim(0, 1.6)
    ax.set_xticks([0, 120, 240, 360, 480, 600])
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, format='pdf')
    plt.close()
    print(f"    Saved: {out_path}")


def plot_summary(mode, matrix, meta, plot_dir):
    x     = np.linspace(0, MAX_DURATION, RESOLUTION)
    color = MODE_COLORS.get(mode, "grey")

    all_selected = []
    for seed in SEEDS:
        all_selected.extend(sample_indices(meta, seed))

    subset     = matrix[all_selected]
    mean_curve = np.nanmean(subset, axis=0)
    std_curve  = np.nanstd(subset, axis=0)
    n          = len(all_selected)

    fig, ax = plt.subplots(figsize=(10, 5))

    if SHOW_INDIVIDUAL_RUNS:
        for i in all_selected:
            ax.plot(x, matrix[i], color=color, alpha=0.05, linewidth=0.6)
    if SHOW_STD_BAND:
        ax.fill_between(x,
                        mean_curve - STD_MULTIPLIER * std_curve,
                        mean_curve + STD_MULTIPLIER * std_curve,
                        color=color, alpha=0.2, linewidth=0)
    if SHOW_MEAN:
        ax.plot(x, mean_curve, color=color, alpha=1.0, linewidth=2.5)


    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Normalised distance to router  (d / d₀)')
    ax.set_xlim(0, MAX_DURATION)
    ax.set_ylim(0, 1.6)
    ax.set_xticks([0, 120, 240, 360, 480, 600])
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out_path = os.path.join(plot_dir, f'{mode}_summary_all_seeds.pdf')
    plt.savefig(out_path, format='pdf')
    plt.close()
    print(f'    Saved: {out_path}')


# ══════════════════════════════════════════════════════════════════════════════
# ── Main ──────────────────────────────────────────────────────────────────────
# ══════════════════════════════════════════════════════════════════════════════

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
    os.makedirs(plot_dir, exist_ok=True)

    # ── Part 1: Extract all modes ─────────────────────────────────────────────
    print("\n── Extraction ───────────────────────────────────────────────────────────")
    for mode, folders in EXTRACT_MODES.items():
        extract_mode(session_root, mode, folders, out_dir)

    # ── Part 2: Plot combined modes (yamauchi, gao, rss) ─────────────────────
    print("\n── Plotting (combined modes) ─────────────────────────────────────────────")
    for mode in PLOT_MODES:
        print(f"\n── {mode} ───────────────────────────────────────────────────────────")
        matrix, meta = load_mode(out_dir, mode)
        if matrix is None:
            continue

        if OUTPUT_ALL_RUNS:
            plot_runs(mode, matrix, list(range(len(meta))),
                      title="all runs",
                      out_path=os.path.join(plot_dir, f"{mode}_all_runs.pdf"))
        if OUTPUT_SEEDED:
            for seed in SEEDS:
                indices = sample_indices(meta, seed)
                plot_runs(mode, matrix, indices,
                          title=f"sampled max {MAX_RUNS}/combination  (seed={seed})",
                          out_path=os.path.join(plot_dir, f"{mode}_sampled_seed{seed}.pdf"))
        if OUTPUT_SEEDED:
            indices = first_indices(meta)
            plot_runs(mode, matrix, indices,
                    title=f"first {MAX_RUNS}/combination (deterministic)",
                    out_path=os.path.join(plot_dir, f"{mode}_first{MAX_RUNS}.pdf"))
    
        if OUTPUT_SUMMARY:
            plot_summary(mode, matrix, meta, plot_dir)
        
    # ── 3x1 combined progression subplot ─────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True, sharey=True)
    x = np.linspace(0, MAX_DURATION, RESOLUTION)

    for ax, mode in zip(axes, PLOT_MODES):
        mat, met = load_mode(out_dir, mode)
        if mat is None:
            continue
        color = MODE_COLORS[mode]
        indices = first_indices(met)
        subset = mat[indices]
        mean_curve = np.nanmean(subset, axis=0)
        std_curve  = np.nanstd(subset, axis=0)

        if SHOW_INDIVIDUAL_RUNS:
            for i in indices:
                ax.plot(x, mat[i], color=color, alpha=0.15, linewidth=0.7)
        if SHOW_STD_BAND:
            ax.fill_between(x, mean_curve - STD_MULTIPLIER * std_curve,
                            mean_curve + STD_MULTIPLIER * std_curve,
                            color=color, alpha=0.2, linewidth=0)
        if SHOW_MEAN:
            ax.plot(x, mean_curve, color=color, linewidth=2.5)

        ax.set_xlim(0, MAX_DURATION)
        ax.set_ylim(0, 1.6)
        ax.set_xticks([0, 120, 240, 360, 480, 600])
        ax.set_ylabel('Normalised distance  (d / d₀)')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    out_path = os.path.join(plot_dir, 'combined_3x1.pdf')
    plt.savefig(out_path, format='pdf')
    plt.close()
    print(f"    Saved: {out_path}")

    # ── Stats table: combined modes ──────────────────────────────────────────
    print("\n── Progression statistics (first 3/combination) ──────────────────────────")
    combined_stats = []
    for mode in PLOT_MODES:
        mat, met = load_mode(out_dir, mode)
        if mat is not None:
            s = compute_stats(mat, first_indices(met))
            combined_stats.append((mode, s))
    print_stats_table(combined_stats)

    # ── Part 3: Plot RSS variants ─────────────────────────────────────────────
    if OUTPUT_RSS_VARIANTS:
        print("\n── Plotting (RSS variants) ───────────────────────────────────────────────")
        variants     = ["rss_1", "rss_2", "rss_3", "rss_4"]
        variant_data = {}
        for v in variants:
            mat, met = load_mode(out_dir, v)
            if mat is not None:
                variant_data[v] = (mat, met)

        for v, (mat, met) in variant_data.items():
            print(f"\n── {v} ──────────────────────────────────────────────────────────────")
            if OUTPUT_ALL_RUNS:
                plot_runs(v, mat, list(range(len(met))),
                          title="all runs",
                          out_path=os.path.join(plot_dir, f"{v}_all_runs.pdf"))
            if OUTPUT_SEEDED:
                for seed in SEEDS:
                    indices = sample_indices(met, seed)
                    plot_runs(v, mat, indices,
                              title=f"sampled max {MAX_RUNS}/combination  (seed={seed})",
                              out_path=os.path.join(plot_dir, f"{v}_sampled_seed{seed}.pdf"))
                    
            if OUTPUT_SEEDED:
                indices = first_indices(met)
                plot_runs(v, mat, indices,
                        title=f"first {MAX_RUNS}/combination (deterministic)",
                        out_path=os.path.join(plot_dir, f"{v}_first{MAX_RUNS}.pdf"))
            if OUTPUT_SUMMARY:
                plot_summary(v, mat, met, plot_dir)

        # 2x2 subplot
        if len(variant_data) == 4:
            print("\n  Generating 2x2 RSS variant summary...")
            fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True, sharey=True)
            x = np.linspace(0, MAX_DURATION, RESOLUTION)

            for ax, v in zip(axes.flat, variants):
                if v not in variant_data:
                    ax.set_visible(False)
                    continue
                mat, met = variant_data[v]
                color    = MODE_COLORS[v]

                all_selected = []
                for seed in SEEDS:
                    all_selected.extend(sample_indices(met, seed))

                subset     = mat[all_selected]
                mean_curve = np.nanmean(subset, axis=0)
                std_curve  = np.nanstd(subset, axis=0)

                if SHOW_INDIVIDUAL_RUNS:
                    for i in all_selected:
                        ax.plot(x, mat[i], color=color, alpha=0.04, linewidth=0.6)
                if SHOW_STD_BAND:
                    ax.fill_between(x,
                                    mean_curve - STD_MULTIPLIER * std_curve,
                                    mean_curve + STD_MULTIPLIER * std_curve,
                                    color=color, alpha=0.2, linewidth=0)
                if SHOW_MEAN:
                    ax.plot(x, mean_curve, color=color, linewidth=2.5,
                            label=f'mean ± {STD_MULTIPLIER} std')
                if SHOW_MEAN or SHOW_STD_BAND:
                    ax.legend(loc='upper right', frameon=False, fontsize=8)

                ax.set_title(v, fontsize=11)
                ax.set_xlim(0, MAX_DURATION)
                ax.set_ylim(0, 1.5)
                ax.set_xticks([0, 120, 240, 360, 480, 600])
                ax.grid(True, alpha=0.3)

            for ax in axes[:, 0]:
                ax.set_ylabel('Normalised distance to router  (d / d₀)')
            for ax in axes[1, :]:
                ax.set_xlabel('Time (s)')

            fig.suptitle('RSS variants — noise sensitivity (grand mean across all seeds)', fontsize=12)
            plt.tight_layout()
            out_path = os.path.join(plot_dir, 'rss_variants_2x2.pdf')
            plt.savefig(out_path, format='pdf')
            plt.close()
            print(f'  Saved: {out_path}')

    # ── Stats table: RSS variants ────────────────────────────────────────────
    if OUTPUT_RSS_VARIANTS:
        print("\n── Progression statistics (RSS variants, all usable runs) ────────────────")
        variant_stats = []
        for v in ["rss_1", "rss_2", "rss_3", "rss_4"]:
            mat, met = load_mode(out_dir, v)
            if mat is not None:
                s = compute_stats(mat, list(range(len(met))))
                variant_stats.append((v, s))
        print_stats_table(variant_stats)

    print("\nDone.")


if __name__ == '__main__':
    main()