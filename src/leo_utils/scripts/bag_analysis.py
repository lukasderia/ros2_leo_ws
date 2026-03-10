#!/usr/bin/env python3
"""
ROS2 Bag Analysis Script for RSS-Guided Frontier Exploration
Reads bag directly via sqlite3 - no rosbag2_py required.

Usage:
    python3 analyze_bag.py <path_to_bag_directory> [router_x] [router_y]

Requirements:
    source /opt/ros/foxy/setup.bash
    source ~/ros2_leo_ws/install/setup.bash
"""

import sys
import os
import re
import glob
import sqlite3
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.collections import LineCollection
import warnings
warnings.filterwarnings('ignore')

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("ERROR: rclpy not found. Source ROS2 first:")
    print("  source /opt/ros/foxy/setup.bash")
    print("  source ~/ros2_leo_ws/install/setup.bash")
    sys.exit(1)


# ─── BAG READER ───────────────────────────────────────────────────────────────

def find_db3(bag_path):
    if bag_path.endswith('.db3'):
        return bag_path
    matches = glob.glob(os.path.join(bag_path, '*.db3'))
    if not matches:
        print(f"ERROR: No .db3 file found in {bag_path}")
        sys.exit(1)
    return matches[0]

def parse_router_from_name(bag_path):
    name = os.path.basename(bag_path.rstrip('/'))
    match = re.search(r'_rx([-\d.]+)_ry([-\d.]+)_bx', name)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None, None


def read_topics(bag_path, topic_names):
    db3 = find_db3(bag_path)
    conn = sqlite3.connect(db3)

    cur = conn.execute("SELECT name, type FROM topics")
    type_map = {row[0]: row[1] for row in cur.fetchall()}

    available = {t for t in topic_names if t in type_map}
    for t in topic_names:
        if t not in type_map:
            print(f"  WARNING: '{t}' not in bag, skipping")

    cur = conn.execute("SELECT id, name FROM topics")
    id_to_name = {row[0]: row[1] for row in cur.fetchall()}
    wanted_ids = {tid for tid, name in id_to_name.items() if name in available}

    data = {t: [] for t in topic_names}
    if not wanted_ids:
        conn.close()
        return data

    placeholders = ','.join('?' * len(wanted_ids))
    cur = conn.execute(
        f"SELECT topic_id, timestamp, data FROM messages "
        f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp",
        list(wanted_ids)
    )

    for topic_id, timestamp, raw in cur.fetchall():
        topic_name = id_to_name[topic_id]
        if topic_name not in available:
            continue
        try:
            msg_type = get_message(type_map[topic_name])
            msg = deserialize_message(bytes(raw), msg_type)
            data[topic_name].append((timestamp, msg))
        except Exception:
            pass

    conn.close()
    return data


# ─── TF: compose map->odom->base_link ────────────────────────────────────────

def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)]
    ])


def compose_transforms(t1_xyz, q1, t2_xyz, q2):
    R1   = quat_to_mat(q1)
    R2   = quat_to_mat(q2)
    t_out = R1 @ np.array(t2_xyz) + np.array(t1_xyz)
    R_out = R1 @ R2
    tr = R_out[0,0] + R_out[1,1] + R_out[2,2]
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
    Returns array of (t_sec, x, y, yaw) sorted by time.
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
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        trajectory.append((ts / 1e9, t_out[0], t_out[1], yaw, np.degrees(roll), np.degrees(pitch)))

    trajectory.sort(key=lambda x: x[0])
    # Downsample to every 5th point for plotting
    return np.array(trajectory[::5])


def interpolate_pose(trajectory, timestamps):
    """
    Given trajectory (N,4) array [t, x, y, yaw] and query timestamps,
    return interpolated (x, y) for each query timestamp.
    """
    traj_t = trajectory[:, 0]
    x_interp = np.interp(timestamps, traj_t, trajectory[:, 1])
    y_interp = np.interp(timestamps, traj_t, trajectory[:, 2])
    return x_interp, y_interp


# ─── MAP ──────────────────────────────────────────────────────────────────────

def map_coverage_over_time(map_messages):
    times, coverage = [], []
    for ts, msg in map_messages:
        data = np.array(msg.data)
        known = np.sum(data >= 0)
        area  = known * msg.info.resolution ** 2
        times.append(ts / 1e9)
        coverage.append(area)
    return np.array(times), np.array(coverage)


def map_to_image(map_msg):
    w, h = map_msg.info.width, map_msg.info.height
    data = np.array(map_msg.data).reshape(h, w)
    img  = np.full((h, w, 3), 180, dtype=np.uint8)
    img[data == 0]  = [235, 235, 235]
    img[data > 50]  = [20,  20,  20]
    return img, map_msg.info


# ─── RSS ──────────────────────────────────────────────────────────────────────

def extract_rss_cloud(rss_messages):
    """Return (timestamps_sec, x, y, rss) from all messages."""
    all_ts, all_x, all_y, all_rss = [], [], [], []
    for ts, msg in rss_messages:
        ps  = msg.point_step
        raw = bytes(msg.data)
        # Only read the newest point (last) from each message to get time series
        if msg.width == 0:
            continue
        i   = msg.width - 1
        off = i * ps
        x   = struct.unpack_from('<f', raw, off)[0]
        y   = struct.unpack_from('<f', raw, off + 4)[0]
        rss = struct.unpack_from('<f', raw, off + 12)[0]
        all_ts.append(ts / 1e9)
        all_x.append(x)
        all_y.append(y)
        all_rss.append(rss)
    return np.array(all_ts), np.array(all_x), np.array(all_y), np.array(all_rss)


def extract_rss_cloud_final(rss_messages):
    """Return full accumulated cloud from last message."""
    if not rss_messages:
        return np.array([]), np.array([]), np.array([])
    _, msg = rss_messages[-1]
    points = []
    ps  = msg.point_step
    raw = bytes(msg.data)
    for i in range(msg.width):
        off = i * ps
        x   = struct.unpack_from('<f', raw, off)[0]
        y   = struct.unpack_from('<f', raw, off + 4)[0]
        rss = struct.unpack_from('<f', raw, off + 12)[0]
        points.append((x, y, rss))
    if not points:
        return np.array([]), np.array([]), np.array([])
    pts = np.array(points)
    return pts[:, 0], pts[:, 1], pts[:, 2]


def extract_gradient_over_time(grad_messages):
    """Return (t, gx_norm, gy_norm, raw_mag) dropping outlier spikes."""
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

    # Drop spike outliers: anything above 10x median magnitude
    mags   = np.array([r[3] for r in result])
    median = np.median(mags[mags > 1e-6]) if np.any(mags > 1e-6) else 1.0
    result = [r for r in result if r[3] <= 10.0 * median]
    return result


# ─── VELOCITY ─────────────────────────────────────────────────────────────────

def extract_velocity(cmdvel_messages):
    times, linear, angular = [], [], []
    for ts, msg in cmdvel_messages:
        times.append(ts / 1e9)
        linear.append(msg.linear.x)
        angular.append(msg.angular.z)
    return np.array(times), np.array(linear), np.array(angular)


# ─── GOALS ────────────────────────────────────────────────────────────────────

def extract_goals(goal_messages):
    return [(ts / 1e9, msg.pose.position.x, msg.pose.position.y)
            for ts, msg in goal_messages]


# ─── METRICS ──────────────────────────────────────────────────────────────────

def compute_path_length(trajectory):
    if len(trajectory) < 2:
        return 0.0
    pts = trajectory[:, 1:3]
    return float(np.sum(np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1))))


def distance_to_router(trajectory, rx, ry):
    dx = trajectory[:, 1] - rx
    dy = trajectory[:, 2] - ry
    return trajectory[:, 0], np.sqrt(dx**2 + dy**2)


def find_discovery_time(trajectory, rx, ry, threshold=1.5):
    dx = trajectory[:, 1] - rx
    dy = trajectory[:, 2] - ry
    dists = np.sqrt(dx**2 + dy**2)
    idx   = np.where(dists < threshold)[0]
    return trajectory[idx[0], 0] if len(idx) > 0 else None


def angle_diff(a, b):
    """Signed angle difference a-b wrapped to [-pi, pi]."""
    d = a - b
    return (d + np.pi) % (2 * np.pi) - np.pi


# ─── TRAJECTORY PLOT ──────────────────────────────────────────────────────────

def plot_trajectory_with_arrows(ax, trajectory, t_min, t_max, arrow_every=30):
    arr = trajectory
    points   = arr[:, 1:3].reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap='plasma',
                        norm=plt.Normalize(t_min, t_max),
                        linewidth=2.0, zorder=3)
    lc.set_array(arr[:-1, 0])
    ax.add_collection(lc)

    # for i in range(0, len(arr) - arrow_every, arrow_every):
    #     x, y, yaw = arr[i, 1], arr[i, 2], arr[i, 3]
    #     ax.annotate('', xy=(x + 0.4*np.cos(yaw), y + 0.4*np.sin(yaw)),
    #                 xytext=(x, y),
    #                 arrowprops=dict(arrowstyle='->', color='white',
    #                                 lw=1.5, mutation_scale=12),
    #                 zorder=5)
    return lc


# ─── MAIN ─────────────────────────────────────────────────────────────────────

def plot_all(bag_path, router_x, router_y):
    print(f"\nAnalyzing: {bag_path}")
    print(f"Router position: ({router_x}, {router_y})\n")

    topics_needed = ['/tf', '/map', '/rss', '/rss_gradient',
                     '/cmd_vel', '/goal_pose', '/weak_signal_state']

    print("Reading bag (may take a moment)...")
    data = read_topics(bag_path, topics_needed)
    for t, msgs in data.items():
        print(f"  {t:<35} {len(msgs)} messages")

    print("\nProcessing transforms...")
    trajectory = extract_robot_trajectory(data['/tf'])
    print(f"  Trajectory points: {len(trajectory)}")

    map_times, cov   = map_coverage_over_time(data['/map'])
    latest_map       = data['/map'][-1][1] if data['/map'] else None
    hx, hy, hrss     = extract_rss_cloud_final(data['/rss'])
    rss_ts, rss_x, rss_y, rss_val = extract_rss_cloud(data['/rss'])
    gradients        = extract_gradient_over_time(data['/rss_gradient'])
    goals            = extract_goals(data['/goal_pose'])

    vel_times = lin_vel = ang_vel = np.array([])
    if data['/cmd_vel']:
        vel_times, lin_vel, ang_vel = extract_velocity(data['/cmd_vel'])

    # Normalize to t=0
    t0 = trajectory[0, 0] if len(trajectory) > 0 else 0.0
    if len(trajectory):
        trajectory[:, 0] -= t0
    map_times  = map_times  - t0 if len(map_times)  else map_times
    vel_times  = vel_times  - t0 if len(vel_times)  else vel_times
    rss_ts     = rss_ts     - t0 if len(rss_ts)     else rss_ts
    goals      = [(t - t0, x, y) for t, x, y in goals]
    gradients  = [(t - t0, gx, gy, mag) for t, gx, gy, mag in gradients]

    # Metrics
    duration    = trajectory[-1, 0] if len(trajectory) else 0
    path_length = compute_path_length(trajectory)
    avg_speed   = path_length / duration if duration > 0 else 0
    final_cov   = float(cov[-1]) if len(cov) > 0 else 0

    dist_times = dist_arr = np.array([])
    disc_time  = None
    if len(trajectory):
        dist_times, dist_arr = distance_to_router(trajectory, router_x, router_y)
        disc_time = find_discovery_time(trajectory, router_x, router_y)

    print(f"\n{'='*50}")
    print(f"  Duration:          {duration:.1f} s  ({duration/60:.1f} min)")
    print(f"  Path length:       {path_length:.2f} m")
    print(f"  Avg speed:         {avg_speed:.3f} m/s")
    print(f"  Final coverage:    {final_cov:.1f} m²")
    print(f"  Goals published:   {len(goals)}")
    print(f"  RSS measurements:  {len(rss_x)}")
    if disc_time is not None:
        print(f"  Source found at:   {disc_time:.1f} s")
    else:
        print(f"  Source found:      NOT REACHED")
    if len(dist_arr):
        print(f"  Min dist to router:{dist_arr.min():.2f} m")
    print(f"{'='*50}\n")

    # ── Gradient vs true bearing ──────────────────────────────────────────────
    # Interpolate robot pose at each gradient timestamp
    grad_bearing_error = []
    if gradients and len(trajectory) > 1:
        grad_arr  = np.array(gradients)   # (t, gx, gy, mag)
        gt        = grad_arr[:, 0]
        robot_gx, robot_gy = interpolate_pose(trajectory, gt)

        true_bearing  = np.arctan2(router_y - robot_gy, router_x - robot_gx)
        grad_bearing  = np.arctan2(grad_arr[:, 2], grad_arr[:, 1])
        error_deg     = np.degrees(angle_diff(grad_bearing, true_bearing))
        grad_bearing_error = (gt, np.degrees(true_bearing),
                              np.degrees(grad_bearing), error_deg)

    # ── Map image ─────────────────────────────────────────────────────────────
    img = ox = oy = mw = mh = None
    if latest_map:
        img, info = map_to_image(latest_map)
        ox = info.origin.position.x
        oy = info.origin.position.y
        mw = info.width  * info.resolution
        mh = info.height * info.resolution

    # ── Layout: 8 rows x 2 cols ───────────────────────────────────────────────
    # Left col:  trajectory (rows 0-2), heatmap (rows 3-5), summary (rows 6-7)
    # Right col: coverage (rows 0-1), distance (rows 2-3), grad mag (rows 4-5), grad dir (rows 6-7)
    fig = plt.figure(figsize=(22, 40))
    fig.suptitle('RSS-Guided Frontier Exploration — Run Analysis', fontsize=16, y=0.995)
    gs  = GridSpec(8, 2, figure=fig, hspace=0.35, wspace=0.30)

    t_min = 0
    t_max = duration

    # ── LEFT COL ──────────────────────────────────────────────────────────────

    # Trajectory — rows 0-2, col 0
    ax_traj = fig.add_subplot(gs[0:3, 0])
    ax_traj.set_title('Robot Trajectory\n(color = time, arrows = heading)', fontsize=11)
    if img is not None:
        ax_traj.imshow(img, origin='lower', extent=[ox,ox+mw,oy,oy+mh], aspect='equal')
    if len(trajectory):
        lc = plot_trajectory_with_arrows(ax_traj, trajectory, t_min, t_max, arrow_every=25)
        sm = plt.cm.ScalarMappable(cmap='plasma', norm=plt.Normalize(t_min, t_max))
        sm.set_array([])
        plt.colorbar(sm, ax=ax_traj, label='Time (s)', fraction=0.04, pad=0.02)
        ax_traj.plot(trajectory[0,1],  trajectory[0,2],  'go', ms=10, zorder=6, label='Start')
        ax_traj.plot(trajectory[-1,1], trajectory[-1,2], 'r^', ms=10, zorder=6, label='End')
    # if goals:
    #     ax_traj.scatter([g[1] for g in goals], [g[2] for g in goals],
    #                     c='cyan', s=40, marker='x', linewidths=2,
    #                     zorder=5, label='Goals', alpha=0.9)
    ax_traj.plot(router_x, router_y, 'r*', ms=18, zorder=7,
                 label=f'Router ({router_x:.0f},{router_y:.0f})')
    ax_traj.set_xlabel('X (m)'); ax_traj.set_ylabel('Y (m)')
    ax_traj.legend(loc='upper right', fontsize=8)
    ax_traj.grid(True, alpha=0.3)

    # Heatmap — rows 3-5, col 0
    ax_rss = fig.add_subplot(gs[3:6, 0])
    ax_rss.set_title('RSS Signal Heatmap\n(red = strong, blue = weak)', fontsize=11)
    if img is not None:
        ax_rss.imshow(img, origin='lower', extent=[ox,ox+mw,oy,oy+mh], aspect='equal', alpha=0.5)
    if len(hx) > 0:
        sc = ax_rss.scatter(hx, hy, c=hrss, cmap='coolwarm',
                            s=8, alpha=0.85, zorder=3)
        plt.colorbar(sc, ax=ax_rss, label='RSS value', fraction=0.04, pad=0.02)
    ax_rss.plot(router_x, router_y, 'r*', ms=18, zorder=5, label='Router')
    ax_rss.legend(fontsize=8)
    ax_rss.set_xlabel('X (m)'); ax_rss.set_ylabel('Y (m)')
    ax_rss.grid(True, alpha=0.3)

    # Summary — rows 6-7, col 0
    ax_sum = fig.add_subplot(gs[6:8, 0])
    ax_sum.axis('off')
    ax_sum.set_title('Run Summary', fontsize=11)
    lines = [
        f"Duration:        {duration:.1f} s ({duration/60:.1f} min)",
        f"Path length:     {path_length:.2f} m",
        f"Avg speed:       {avg_speed:.3f} m/s",
        f"Final coverage:  {final_cov:.1f} m²",
        f"Goals published: {len(goals)}",
        f"RSS measurements:{len(rss_x)}",
        f"",
        f"Router:          ({router_x:.1f}, {router_y:.1f})",
        f"Source found:    {'%.1f s' % disc_time if disc_time is not None else 'NOT REACHED'}",
    ]
    if len(dist_arr):
        lines.append(f"Min dist to src: {dist_arr.min():.2f} m")
    if len(lin_vel):
        moving = lin_vel[lin_vel > 0.01]
        if len(moving):
            lines.append(f"Avg moving vel:  {moving.mean():.3f} m/s")
    ax_sum.text(0.05, 0.95, '\n'.join(lines), transform=ax_sum.transAxes,
                va='top', fontsize=11, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

    # ── RIGHT COL ─────────────────────────────────────────────────────────────

    # Rows 0-1 right — Map coverage
    ax_cov = fig.add_subplot(gs[0:2, 1])
    ax_cov.set_title('Map Coverage Over Time')
    if len(map_times) > 0:
        ax_cov.plot(map_times, cov, 'b-', linewidth=2)
        ax_cov.fill_between(map_times, cov, alpha=0.15)
    ax_cov.set_xlabel('Time (s)'); ax_cov.set_ylabel('Explored Area (m²)')
    ax_cov.grid(True, alpha=0.3)

    # Rows 2-3 right — Distance to router
    ax_dist = fig.add_subplot(gs[2:4, 1])
    ax_dist.set_title('Distance to Router Over Time')
    if len(dist_times) > 0:
        ax_dist.plot(dist_times, dist_arr, 'r-', linewidth=2)
        ax_dist.axhline(y=1.5, color='g', linestyle='--', label='Threshold (1.5m)')
        if disc_time is not None:
            ax_dist.axvline(x=disc_time, color='orange', linestyle='--',
                            label=f'Found @ {disc_time:.0f}s')
    ax_dist.set_xlabel('Time (s)'); ax_dist.set_ylabel('Distance (m)')
    ax_dist.legend(fontsize=8); ax_dist.grid(True, alpha=0.3)

    # Rows 4-5 right — 
    ax_orient = fig.add_subplot(gs[4:6, 1])
    ax_orient.set_title('Robot Orientation Over Time\n(flat = normal, >45° = tipping, >90° = flipped)')
    if len(trajectory) > 0:
        ax_orient.plot(trajectory[:, 0], trajectory[:, 4], 'b-', linewidth=1.5, label='Roll')
        ax_orient.plot(trajectory[:, 0], trajectory[:, 5], 'r-', linewidth=1.5, label='Pitch')
        ax_orient.axhline(y=45,  color='orange', linestyle='--', alpha=0.6, label='±45° threshold')
        ax_orient.axhline(y=-45, color='orange', linestyle='--', alpha=0.6)
        ax_orient.axhline(y=90,  color='red',    linestyle='--', alpha=0.6, label='±90° (flipped)')
        ax_orient.axhline(y=-90, color='red',    linestyle='--', alpha=0.6)
        ax_orient.set_ylim(-185, 185)
    ax_orient.set_xlabel('Time (s)'); ax_orient.set_ylabel('Angle (deg)')
    ax_orient.legend(fontsize=8); ax_orient.grid(True, alpha=0.3)

    # Rows 6-7 right — Gradient direction vs true bearing
    ax_bear = fig.add_subplot(gs[6:8, 1])
    ax_bear.set_title('Gradient Direction vs True Bearing to Router')
    if grad_bearing_error:
        gt, true_deg, grad_deg, err_deg = grad_bearing_error
        ax_bear.plot(gt, true_deg, 'b-',  linewidth=2,   label='True bearing')
        ax_bear.plot(gt, grad_deg, 'r--', linewidth=1.5, label='Gradient dir', alpha=0.8)
        ax_twin = ax_bear.twinx()
        ax_twin.plot(gt, err_deg, color='orange', linewidth=1, alpha=0.6, label='Error')
        ax_twin.axhline(y=0,   color='orange', linewidth=0.5, linestyle=':')
        ax_twin.axhline(y=45,  color='orange', linewidth=0.5, linestyle='--', alpha=0.4)
        ax_twin.axhline(y=-45, color='orange', linewidth=0.5, linestyle='--', alpha=0.4)
        ax_twin.set_ylabel('Error (deg)', color='orange')
        ax_twin.tick_params(axis='y', labelcolor='orange')
        ax_twin.set_ylim(-185, 185)
        lines1, labels1 = ax_bear.get_legend_handles_labels()
        lines2, labels2 = ax_twin.get_legend_handles_labels()
        ax_bear.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc='upper right')
    ax_bear.set_xlabel('Time (s)'); ax_bear.set_ylabel('Direction (deg)')
    ax_bear.set_ylim(-185, 185)
    ax_bear.axhline(y=0, color='k', linewidth=0.5)
    ax_bear.grid(True, alpha=0.3)

    bag_name = os.path.basename(bag_path.rstrip('/'))
    out = os.path.join(os.path.dirname(os.path.abspath(bag_path)),
                       f"{bag_name}_analysis.png")
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"Saved: {out}")
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_bag.py <bag_dir> [router_x] [router_y]")
        sys.exit(1)

    bag_path = sys.argv[1]

    if not os.path.exists(bag_path):
        print(f"ERROR: Path not found: {bag_path}")
        sys.exit(1)

    router_x, router_y = parse_router_from_name(bag_path)

    if router_x is None:
        router_x = float(sys.argv[2]) if len(sys.argv) > 2 else -9.0
        router_y = float(sys.argv[3]) if len(sys.argv) > 3 else 9.0
    else:
        print(f"Parsed router position from name: ({router_x}, {router_y})")
        if len(sys.argv) > 2:
            router_x = float(sys.argv[2])
            router_y = float(sys.argv[3])

    plot_all(bag_path, router_x, router_y)