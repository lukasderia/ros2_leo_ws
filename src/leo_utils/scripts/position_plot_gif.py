import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
import json
import re

script_dir = Path(__file__).parent

matrix_rss  = np.load(script_dir / "rss_position_matrix.npy")
matrix_yama = np.load(script_dir / "yamauchi_position_matrix.npy")
matrix_gao  = np.load(script_dir / "gao_position_matrix.npy")
display_map  = np.load(script_dir / "display_map.npy")

COMBO_INDICES = [8, 16, 23]
STEP = 3

MODE_COLORS = {
    'Yamauchi': ['#6baed6', '#2171b5', '#08306b'],
    'Gao':      ['#74c476', '#238b45', '#00441b'],
    'RSS':      ['#fc8d59', '#d7301f', '#7f0000'],
}
MODE_NAMES = ['Yamauchi', 'Gao', 'RSS']
MATRICES   = [matrix_yama, matrix_gao, matrix_rss]


def get_combo_meta(json_path, run_index):
    with open(json_path) as f:
        data = json.load(f)
    entry = data[run_index]
    router_x = entry['router_x']
    router_y  = entry['router_y']
    match = re.search(r'bx([-\d.]+)_by([-\d.]+)', entry['bag_name'])
    robot_x = float(match.group(1))
    robot_y = float(match.group(2))
    return (robot_x, robot_y), (router_x, router_y)


def load_trajectories(matrix, combo_idx, step=STEP):
    runs = [combo_idx * 3 + r for r in range(3)]
    trajs = []
    for r in runs:
        x = matrix[r, ::step, 0]
        y = matrix[r, ::step, 1]
        trajs.append((x, y))
    return trajs


def make_gif(combo_idx, mode_idx):
    mode_name = MODE_NAMES[mode_idx]
    matrix    = MATRICES[mode_idx]
    colors    = MODE_COLORS[mode_name]

    first_run = combo_idx * 3
    start, router = get_combo_meta(script_dir / "yamauchi_position_meta.json", first_run)

    trajs   = load_trajectories(matrix, combo_idx)
    max_len = max(len(x) for x, _ in trajs)

    fig, ax = plt.subplots(figsize=(6, 6))
    fig.patch.set_facecolor('white')
    fig.subplots_adjust(left=0.01, right=0.99, top=0.95, bottom=0.01)

    ax.imshow(display_map, origin='lower', cmap='gray_r',
              vmin=0, vmax=100, extent=[-20, 20, -20, 20])
    ax.plot(*start,  marker='o', color='black', markersize=12, zorder=5)
    ax.plot(*router, marker='*', color='gold',  markersize=20,
            markeredgecolor='black', zorder=5)
    ax.set_aspect('equal')
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(mode_name, fontsize=16, fontweight='bold')

    lines = []
    dots  = []
    for run_idx in range(3):
        ln, = ax.plot([], [], color=colors[run_idx], linewidth=2.0, zorder=3)
        dt, = ax.plot([], [], marker='o', color=colors[run_idx], markersize=10, zorder=4)
        lines.append(ln)
        dots.append(dt)

    def init():
        for ln, dt in zip(lines, dots):
            ln.set_data([], [])
            dt.set_data([], [])
        return lines + dots

    def update(frame):
        for run_idx, (x, y) in enumerate(trajs):
            end   = min(frame + 1, len(x))
            final = end - 1
            lines[run_idx].set_data(x[:end], y[:end])
            dots[run_idx].set_data([x[final]], [y[final]])
        return lines + dots

    ani = animation.FuncAnimation(
        fig, update, frames=max_len,
        init_func=init, blit=True, interval=30
    )

    out_path = script_dir / f"combo_{combo_idx}_{mode_name.lower()}.gif"
    ani.save(out_path, writer='pillow', fps=30, dpi=100)
    print(f"Saved: {out_path}")
    plt.close()


if __name__ == "__main__":
    for combo_idx in COMBO_INDICES:
        for mode_idx in range(3):
            make_gif(combo_idx, mode_idx)