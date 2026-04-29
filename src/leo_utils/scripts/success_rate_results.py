#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# ── CSV path from argument ───────────────────────────────────────────────────
if len(sys.argv) < 2:
    print("Usage: python3 success_rate.py /path/to/metrics.csv")
    sys.exit(1)

CSV_PATH = sys.argv[1]
if not os.path.isfile(CSV_PATH):
    print(f"Error: file not found: {CSV_PATH}")
    sys.exit(1)

# ── Output directory ─────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLOT_DIR   = os.path.join(SCRIPT_DIR, 'plots')
os.makedirs(PLOT_DIR, exist_ok=True)

# ── LaTeX-quality settings ───────────────────────────────────────────────────
plt.rcParams.update({
    'font.family':        'serif',
    'font.size':          11,
    'axes.titlesize':     11,
    'axes.labelsize':     11,
    'xtick.labelsize':    10,
    'ytick.labelsize':    10,
    'legend.fontsize':    10,
    'figure.dpi':         300,
    'savefig.dpi':        300,
    'savefig.bbox':       'tight',
    'savefig.pad_inches': 0.05,
    'axes.spines.top':    False,
    'axes.spines.right':  False,
    'axes.grid':          True,
    'grid.alpha':         0.3,
    'grid.linestyle':     '--',
    'hatch.linewidth':     2.5,
})

FIG_W = 5.787
FIG_H = 3.5

GEO_COLORS         = ['#C0392B', '#E67E22']
RSS_COLORS         = ['#1B4F8A', '#3266AD', '#6AA3D9', '#9DC0E8']
RSS_COMBINED_COLOR = '#3266AD'  # rss_4 color

# ── Load & label ──────────────────────────────────────────────────────────────
df = pd.read_csv(CSV_PATH)

def get_label(row):
    if row['mode'] == 'rss':
        return f"RSS-{int(row['stddev'])}"
    return row['mode'].capitalize()

df['label'] = df.apply(get_label, axis=1)

ORDER_RSS      = ['RSS-1', 'RSS-2', 'RSS-3', 'RSS-4']
ORDER_COMBINED = ['Yamauchi', 'Gao', 'RSS']

# ── Build summary ─────────────────────────────────────────────────────────────
def make_row(dataframe, label, filter_col='label', filter_val=None):
    if filter_val is None:
        filter_val = label
    all_runs  = dataframe[dataframe[filter_col] == filter_val]
    non_flip  = all_runs[all_runs['termination_reason'] != 'flip']
    total_all = len(all_runs)
    total_nf  = len(non_flip)
    success   = (non_flip['termination_reason'] == 'router_found').sum()
    timeout   = (non_flip['termination_reason'] == 'timeout').sum()
    flips     = (all_runs['termination_reason'] == 'flip').sum()
    return {
        'mode':            label,
        'total_all':       total_all,
        'total_nf':        total_nf,
        'success':         success,
        'timeout':         timeout,
        'flips':           flips,
        'success_pct_nf':  100 * success / total_nf  if total_nf  > 0 else 0.0,
        'success_pct_all': 100 * success / total_all if total_all > 0 else 0.0,
        'timeout_pct_all': 100 * timeout / total_all if total_all > 0 else 0.0,
        'flip_pct':        100 * flips   / total_all if total_all > 0 else 0.0,
    }

def build_summary(dataframe, order):
    return pd.DataFrame([make_row(dataframe, lbl) for lbl in order])

# RSS entry uses RSS-4 only
summary_combined = pd.DataFrame([
    make_row(df, 'Yamauchi'),
    make_row(df, 'Gao'),
    make_row(df, 'RSS', filter_col='label', filter_val='RSS-4'),
])
summary_rss = build_summary(df, ORDER_RSS)

print("=== Combined view (flips excluded, RSS = RSS-4) ===")
print(summary_combined[['mode','total_nf','success','timeout','success_pct_nf']].to_string(index=False))
print("\n=== RSS variants (flips excluded) ===")
print(summary_rss[['mode','total_nf','success','timeout','success_pct_nf']].to_string(index=False))

# ── Helpers ───────────────────────────────────────────────────────────────────
PALETTE = {
    'Yamauchi': GEO_COLORS[0],
    'Gao':      GEO_COLORS[1],
    'RSS':      RSS_COMBINED_COLOR,
    'RSS-1':    RSS_COLORS[0],
    'RSS-2':    RSS_COLORS[1],
    'RSS-3':    RSS_COLORS[2],
    'RSS-4':    RSS_COLORS[3],
}

def bar_colors(labels):
    return [PALETTE[l] for l in labels]

def save(fig, path):
    fig.savefig(path, format='pdf')
    plt.close(fig)
    print(f"Saved: {path}")

# ── Plot 1: Yamauchi / Gao / RSS-4 combined, flips excluded only ─────────────
def plot1(summary, order):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    x      = np.arange(len(order))
    vals   = summary['success_pct_nf'].values
    colors = bar_colors(order)

    for xi, c, v in zip(x, colors, vals):
        ax.bar(xi, v, color=c, width=0.5, zorder=3, edgecolor=c, linewidth=0.5)
        ax.text(xi, v + 1.2, f'{v:.1f}%', ha='center', va='bottom', fontsize=7)

    ax.set_xticks(x)
    ax.set_xticklabels(order)
    ax.set_ylabel('Success rate (%)')
    ax.set_ylim(0, 100)
    ax.set_yticks(range(0, 101, 20))
    fig.tight_layout()
    save(fig, os.path.join(PLOT_DIR, 'plot1_combined_no_flip.pdf'))

# ── Plot 3: RSS variants only, flips excluded only ────────────────────────────
def plot3(summary, order):
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    x      = np.arange(len(order))
    vals   = summary['success_pct_nf'].values
    colors = bar_colors(order)

    for xi, c, v in zip(x, colors, vals):
        ax.bar(xi, v, color=c, width=0.5, zorder=3, edgecolor=c, linewidth=0.5)
        ax.text(xi, v + 1.2, f'{v:.1f}%', ha='center', va='bottom', fontsize=7)

    ax.set_xticks(x)
    ax.set_xticklabels(order)
    ax.set_ylabel('Success rate (%)')
    ax.set_ylim(0, 100)
    ax.set_yticks(range(0, 101, 20))
    fig.tight_layout()
    save(fig, os.path.join(PLOT_DIR, 'plot3_rss_variants_no_flip.pdf'))

# ── Run ───────────────────────────────────────────────────────────────────────
plot1(summary_combined, ORDER_COMBINED)
plot3(summary_rss,      ORDER_RSS)

print("\nDone. All plots saved to:", PLOT_DIR)