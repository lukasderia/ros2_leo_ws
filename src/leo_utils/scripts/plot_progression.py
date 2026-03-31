#!/usr/bin/env python3
"""
Quick sanity-check plot for progression matrices.
Usage:
    python3 plot_progression_quick.py yamauchi
    python3 plot_progression_quick.py gao
    python3 plot_progression_quick.py rss
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Usage: python3 plot_progression_quick.py <mode>")
    sys.exit(1)

mode = sys.argv[1]
npy_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{mode}_progression.npy")

if not os.path.exists(npy_path):
    print(f"ERROR: {npy_path} not found")
    sys.exit(1)

matrix = np.load(npy_path)
print(f"Loaded {mode}: {matrix.shape[0]} runs x {matrix.shape[1]} time steps")

x = np.linspace(0, 1, matrix.shape[1])

median = np.percentile(matrix, 50, axis=0)
p25    = np.percentile(matrix, 25, axis=0)
p75    = np.percentile(matrix, 75, axis=0)

fig, ax = plt.subplots(figsize=(10, 6))

# # Individual runs
# for i, row in enumerate(matrix):
#     ax.plot(x, row, color='steelblue', alpha=0.3, linewidth=0.8)

# Median and IQR band
ax.fill_between(x, p25, p75, alpha=0.3, color='steelblue', label='IQR (25–75%)')
ax.plot(x, median, color='steelblue', linewidth=2.5, label='Median')

# Reference line: perfect straight-line progression
ax.plot([0, 1], [1, 0], 'k--', linewidth=1, alpha=0.4, label='Perfect progression')

ax.set_xlabel('Normalized time')
ax.set_ylabel('Normalized distance to router')
ax.set_title(f'{mode} — progression plot ({matrix.shape[0]} successful runs)')
ax.set_xlim(0, 1)
ax.set_ylim(0, 1.1)
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()