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
RSS_COMBINED_COLOR = '#3266AD'
SUCCESS_C          = '#1D9E75'
TIMEOUT_C          = '#B4B2A9'
FLIP_C             = '#C0392B'

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

summary_combined = pd.DataFrame([
    make_row(df, 'Yamauchi'),
    make_row(df, 'Gao'),
    make_row(df, 'RSS', filter_col='mode', filter_val='rss'),
])
summary_rss = build_summary(df, ORDER_RSS)

print("=== Combined view (flips excluded) ===")
print(summary_combined[['mode','total_nf','success','timeout','success_pct_nf']].to_string(index=False))
print("\n=== Combined view (flips included) ===")
print(summary_combined[['mode','total_all','success','flips','success_pct_all','flip_pct']].to_string(index=False))
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

def lighten(hex_color, factor=0.5):
    """Return a lighter version of a hex color by blending toward white."""
    hex_color = hex_color.lstrip('#')
    r, g, b = [int(hex_color[i:i+2], 16) for i in (0, 2, 4)]
    r = int(r + (255 - r) * factor)
    g = int(g + (255 - g) * factor)
    b = int(b + (255 - b) * factor)
    return f'#{r:02x}{g:02x}{b:02x}'

def horizontal_legend(fig, ax, items):
    """Horizontal coloured-square legend placed below the x-axis."""
    handles = [plt.Rectangle((0,0), 1, 1, color=c, label=l) for l, c in items]
    ax.legend(
        handles=handles,
        loc='upper center',
        bbox_to_anchor=(0.5, -0.15),
        ncol=len(items),
        frameon=False,
        handlelength=0.8,
        handleheight=0.8,
        fontsize=9,
    )
    fig.subplots_adjust(bottom=0.2)

def save(fig, path):
    fig.savefig(path, format='pdf')
    plt.close(fig)
    print(f"Saved: {path}")

# ── Plot 1: Yamauchi / Gao / RSS combined, grouped bar (excl vs incl flips) ──
def plot1(summary, order):
    from matplotlib.patches import Patch
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    x        = np.arange(len(order))
    vals_nf  = summary['success_pct_nf'].values
    vals_all = summary['success_pct_all'].values
    colors   = bar_colors(order)
    w = 0.5
    offset = w / 3

    # Flips-included bars drawn first (behind), starting at same x, offset right
    for xi, c, v in zip(x, colors, vals_all):
        lc = lighten(c, 0.45)
        ax.bar(xi + offset/2, v, color=lc, width=w, zorder=2,
               edgecolor=c, linewidth=0.0, hatch='////')
        ax.text(xi + offset/2 + w/2 + 0.01, v, f'{v:.1f}%',
                ha='left', va='center', fontsize=7, color='#888888')

    # Flips-excluded bars drawn on top (in front)
    for xi, c, v in zip(x, colors, vals_nf):
        ax.bar(xi - offset/2, v, color=c, width=w, zorder=3,
               edgecolor=c, linewidth=0.5)
        ax.text(xi - offset/2, v + 1.2, f'{v:.1f}%',
                ha='center', va='bottom', fontsize=7)

    ax.set(xticks=x, xticklabels=order, ylabel='Success rate (%)',
           ylim=(0, 100), yticks=range(0, 101, 20))
    ax.set_title('Success rate by exploration strategy')

    handles = [
        Patch(facecolor='#555555', edgecolor='#555555', label='Flips excluded'),
        Patch(facecolor='#cccccc', edgecolor='#555555', hatch='////', label='Flips included'),
    ]
    ax.legend(handles=handles, loc='upper left', frameon=False, fontsize=8)
    fig.tight_layout()
    save(fig, os.path.join(PLOT_DIR, 'plot1_combined_no_flip.pdf'))

# ── Plot 2: Yamauchi / Gao / RSS combined, donut chart with flip ─────────────
def plot2(summary, order):
    n = len(order)
    # Two donuts per row — layout depends on count
    ncols = min(n, 3)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(FIG_W, FIG_W * nrows / ncols))

    # Flatten axes and hide any extras
    axes_flat = np.array(axes).flatten()
    for ax in axes_flat[n:]:
        ax.set_visible(False)

    for i, (ax, label) in enumerate(zip(axes_flat[:n], order)):
        row = summary[summary['mode'] == label].iloc[0]
        s = row['success_pct_all']
        t = row['timeout_pct_all']
        f = row['flip_pct']
        sizes  = [s, t, f]
        colors = [SUCCESS_C, TIMEOUT_C, FLIP_C]

        wedges, _ = ax.pie(
            sizes, colors=colors, startangle=90,
            wedgeprops=dict(width=0.28, edgecolor='white', linewidth=0.8),
            counterclock=False,
        )
        # Centre label: strategy name + total runs
        total = int(row['total_all'])
        ax.text(0, 0.08, label, ha='center', va='center',
                fontsize=9, fontweight='bold')
        ax.text(0, -0.18, f'n={total}', ha='center', va='center',
                fontsize=8, color='#666666')

        # Percentage annotations outside the ring
        cumulative = 0
        for val, color in zip(sizes, colors):
            if val >= 5:
                angle = 90 - (cumulative + val / 2) * 3.6
                rad = np.deg2rad(angle)
                rx, ry = 1.22 * np.cos(rad), 1.22 * np.sin(rad)
                ax.text(rx, ry, f'{val:.0f}%', ha='center', va='center',
                        fontsize=7.5, color='black')
            cumulative += val

    # Shared legend below
    handles = [plt.Rectangle((0,0),1,1, color=c, label=l)
               for l, c in [('Success', SUCCESS_C), ('Timeout', TIMEOUT_C), ('Flip', FLIP_C)]]
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False,
               fontsize=9, bbox_to_anchor=(0.5, -0.02),
               handlelength=0.8, handleheight=0.8)
    fig.suptitle('Run outcomes by exploration strategy (all runs)', fontsize=11)
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.12)
    save(fig, os.path.join(PLOT_DIR, 'plot2_combined_with_flip.pdf'))

# ── Plot 3: RSS variants only, grouped bar (excl vs incl flips) ──────────────
def plot3(summary, order):
    from matplotlib.patches import Patch
    fig, ax = plt.subplots(figsize=(FIG_W, FIG_H))
    x        = np.arange(len(order))
    vals_nf  = summary['success_pct_nf'].values
    vals_all = summary['success_pct_all'].values
    colors   = bar_colors(order)
    w = 0.5
    offset = w / 3

    # Flips-included bars drawn first (behind)
    for xi, c, v in zip(x, colors, vals_all):
        lc = lighten(c, 0.45)
        ax.bar(xi + offset/2, v, color=lc, width=w, zorder=2,
               edgecolor=c, linewidth=0.0, hatch='////')
        ax.text(xi + offset/2 + w/2 + 0.01, v, f'{v:.1f}%',
                ha='left', va='center', fontsize=7, color='#888888')

    # Flips-excluded bars drawn on top (in front)
    for xi, c, v in zip(x, colors, vals_nf):
        ax.bar(xi - offset/2, v, color=c, width=w, zorder=3,
               edgecolor=c, linewidth=0.5)
        ax.text(xi - offset/2, v + 1.2, f'{v:.1f}%',
                ha='center', va='bottom', fontsize=7)



    ax.set(xticks=x, xticklabels=order, ylabel='Success rate (%)',
           ylim=(0, 100), yticks=range(0, 101, 20))
    ax.set_title('RSS variant success rate — noise sensitivity')

    rep_c = colors[0]
    handles = [
        Patch(facecolor=rep_c, edgecolor=rep_c, label='Flips excluded'),
        Patch(facecolor=lighten(rep_c, 0.45), edgecolor=rep_c, hatch='////', label='Flips included'),
    ]
    ax.legend(handles=handles, loc='upper left', frameon=False, fontsize=8)
    fig.tight_layout()
    save(fig, os.path.join(PLOT_DIR, 'plot3_rss_variants_no_flip.pdf'))

# ── Plot 4: RSS variants only, donut chart with flip ─────────────────────────
def plot4(summary, order):
    n = len(order)
    fig, axes = plt.subplots(1, n, figsize=(FIG_W, FIG_W / n * 1.3))

    for ax, label in zip(axes, order):
        row = summary[summary['mode'] == label].iloc[0]
        s = row['success_pct_all']
        t = row['timeout_pct_all']
        f = row['flip_pct']
        sizes  = [s, t, f]
        colors = [SUCCESS_C, TIMEOUT_C, FLIP_C]

        ax.pie(
            sizes, colors=colors, startangle=90,
            wedgeprops=dict(width=0.28, edgecolor='white', linewidth=0.8),
            counterclock=False,
        )
        total = int(row['total_all'])
        ax.text(0, 0.08, label, ha='center', va='center',
                fontsize=9, fontweight='bold')
        ax.text(0, -0.18, f'n={total}', ha='center', va='center',
                fontsize=8, color='#666666')

        cumulative = 0
        for val, color in zip(sizes, colors):
            if val >= 5:
                angle = 90 - (cumulative + val / 2) * 3.6
                rad = np.deg2rad(angle)
                rx, ry = 1.22 * np.cos(rad), 1.22 * np.sin(rad)
                ax.text(rx, ry, f'{val:.0f}%', ha='center', va='center',
                        fontsize=7.5, color='black')
            cumulative += val

    handles = [plt.Rectangle((0,0),1,1, color=c, label=l)
               for l, c in [('Success', SUCCESS_C), ('Timeout', TIMEOUT_C), ('Flip', FLIP_C)]]
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False,
               fontsize=9, bbox_to_anchor=(0.5, -0.04),
               handlelength=0.8, handleheight=0.8)
    fig.suptitle('RSS variant run outcomes — noise sensitivity (all runs)', fontsize=11)
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    save(fig, os.path.join(PLOT_DIR, 'plot4_rss_variants_with_flip.pdf'))

# ── Run ───────────────────────────────────────────────────────────────────────
plot1(summary_combined, ORDER_COMBINED)
plot2(summary_combined, ORDER_COMBINED)
plot3(summary_rss,      ORDER_RSS)
plot4(summary_rss,      ORDER_RSS)

print("\nDone. All plots saved to:", PLOT_DIR)