#!/usr/bin/env python3
"""
Quick sanity-check plot for progression matrices.

Usage:
    python3 plot_progression_quick.py <mode> [termination types...]

Examples:
    python3 plot_progression_quick.py yamauchi
    python3 plot_progression_quick.py yamauchi success
    python3 plot_progression_quick.py rss success timeout
    python3 plot_progression_quick.py gao success timeout flip

If no termination types are given, all runs are included.
Valid types: success, timeout, flip
"""

import sys
import os
import json
import numpy as np
import matplotlib.pyplot as plt

MAX_DURATION = 600.0

VALID_TYPES = {"success", "timeout", "flip"}

# Map friendly CLI names to JSON values
TYPE_MAP = {
    "success": "router_found",
    "timeout": "timeout",
    "flip":    "flip",
}

COLORS = {
    "success": "steelblue",
    "timeout": "orange",
    "flip":    "red",
}

if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

mode = sys.argv[1]

# Parse requested types, default to all
requested = sys.argv[2:] if len(sys.argv) > 2 else list(VALID_TYPES)
invalid = [r for r in requested if r not in VALID_TYPES]
if invalid:
    print(f"ERROR: unknown termination type(s): {invalid}")
    print(f"Valid types: {sorted(VALID_TYPES)}")
    sys.exit(1)

base_dir  = os.path.dirname(os.path.abspath(__file__))
npy_path  = os.path.join(base_dir, f"{mode}_progression.npy")
meta_path = os.path.join(base_dir, f"{mode}_progression_meta.json")

if not os.path.exists(npy_path):
    print(f"ERROR: {npy_path} not found")
    sys.exit(1)

matrix = np.load(npy_path)
with open(meta_path) as f:
    meta = json.load(f)

terminations = [m["termination"] for m in meta]

# Build index lists for each type
indices = {
    t: [i for i, term in enumerate(terminations) if term == TYPE_MAP[t]]
    for t in VALID_TYPES
}

print(f"Loaded {mode}: {matrix.shape[0]} total runs")
for t in VALID_TYPES:
    marker = " <-- included" if t in requested else ""
    print(f"  {t:10s}: {len(indices[t])}{marker}")

# Collect all included rows for median/IQR
included_idx = []
for t in requested:
    included_idx.extend(indices[t])

if not included_idx:
    print("ERROR: no runs match the requested types")
    sys.exit(1)

included_matrix = matrix[included_idx]

x = np.linspace(0, MAX_DURATION, matrix.shape[1])

fig, ax = plt.subplots(figsize=(12, 6))

# Plot individual runs, excluded types drawn faint grey for context
for t in VALID_TYPES:
    if t not in requested:
        for i in indices[t]:
            ax.plot(x, matrix[i], color='lightgrey', alpha=0.15, linewidth=0.5)

for t in requested:
    for i in indices[t]:
        ax.plot(x, matrix[i], color=COLORS[t], alpha=0.2, linewidth=0.7)

# Median and IQR over included runs only
median = np.nanpercentile(included_matrix, 50, axis=0)
p25    = np.nanpercentile(included_matrix, 25, axis=0)
p75    = np.nanpercentile(included_matrix, 75, axis=0)

ax.fill_between(x, p25, p75, alpha=0.3, color='steelblue', label='IQR (25–75%)')
ax.plot(x, median, color='steelblue', linewidth=2.5, label=f'Median ({len(included_idx)} runs)')

# Reference diagonal
ax.plot([0, MAX_DURATION], [1, 0], 'k--', linewidth=1, alpha=0.4, label='Perfect progression')

# Legend entries
for t in VALID_TYPES:
    alpha = 0.6 if t in requested else 0.2
    label = f'{t} ({len(indices[t])})' + ('' if t in requested else ' [excluded]')
    ax.plot([], [], color=COLORS[t], alpha=alpha, linewidth=1, label=label)

ax.set_xlabel('Time (s)')
ax.set_ylabel('Normalized distance to router  (d / d_initial)')
title_types = ' + '.join(requested)
ax.set_title(f'{mode} — progression plot  [{title_types}]  ({len(included_idx)} runs shown)')
ax.set_xlim(0, MAX_DURATION)
ax.set_ylim(0, 1.2)
ax.set_xticks([0, 120, 240, 360, 480, 600])
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()