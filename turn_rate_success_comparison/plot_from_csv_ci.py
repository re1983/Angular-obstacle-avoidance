"""
Plot success rate vs ROT from an existing CSV (exported by rot_success_comparison.py) with:
- Underscores in scenario names replaced by spaces
- Y-axis extended slightly above 100% (0 to 110%) to avoid clipping at full success
- Optional confidence interval error bars if *_ci_low / *_ci_high columns exist

Usage:
    python3 plot_from_csv_ci.py --csv ../rot_success_ci_1000.csv --save rot_success_ci_1000_replot.png

Arguments:
    --csv           Path to input CSV (required)
    --save          Optional output image path (.png/.pdf/.svg). If omitted, shows interactively.
    --dpi           Figure DPI when saving (default 600)
    --no-ci         Disable drawing CI bars even if columns exist
    --title         Custom plot title (default derived)
    --ylim-max      Upper Y limit (default 110)
"""
from __future__ import annotations
import argparse
from pathlib import Path
import csv
import math
import numpy as np
import os
import matplotlib
# Headless safety: if no DISPLAY, switch to non-interactive backend before importing pyplot
if os.environ.get("DISPLAY", "") == "":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_csv(path: str):
    try:
        with open(path, 'r', newline='') as f:
            reader = csv.reader(f)
            rows = list(reader)
    except FileNotFoundError as e:
        # Provide helpful context & suggestions
        cwd = os.getcwd()
        # Look for similarly named files in CWD
        candidates = [p for p in os.listdir(cwd) if p.endswith('.csv') and 'rot_success' in p]
        hint = ''
        if candidates:
            hint = ('\nDid you mean one of these in the current directory?\n  - ' + '\n  - '.join(candidates))
        else:
            # Search one level deep inside turn_rate_success_comparison folder if exists
            comp_dir = Path(cwd) / 'turn_rate_success_comparison'
            if comp_dir.exists():
                inner = [p.name for p in comp_dir.glob('*.csv') if 'rot_success' in p.name]
                if inner:
                    hint = ('\nCSV not found here, but inside turn_rate_success_comparison there are:\n  - ' + '\n  - '.join(inner))
        raise FileNotFoundError(f"CSV file not found: {path}\nWorking directory: {cwd}{hint}\nIf the file was written to repo root by rot_success_comparison.py, run with --csv rot_success_ci_1000.csv (or the name you used).\nTo regenerate: python3 turn_rate_success_comparison/rot_success_comparison.py --csv rot_success_ci.csv ...") from e
    if not rows:
        raise ValueError('Empty CSV')
    header = rows[0]
    data_rows = rows[1:]
    # Build mapping per scenario
    # header pattern: rot_deg, <Scenario>_success_rate, <Scenario>_ci_low, <Scenario>_ci_high, ...
    scenarios = []
    scenario_fields = {}
    for h in header:
        if h.endswith('_success_rate'):
            sc = h[:-len('_success_rate')]
            scenarios.append(sc)
            scenario_fields[sc] = {
                'rate': h,
                'ci_low': f'{sc}_ci_low',
                'ci_high': f'{sc}_ci_high'
            }
    rot_values = []
    per_scenario = {sc: {'rate': [], 'ci_low': [], 'ci_high': []} for sc in scenarios}
    for r in data_rows:
        if not r or all(s.strip()=='' for s in r):
            continue
        rot = float(r[0])
        rot_values.append(rot)
        header_index = {h:i for i,h in enumerate(header)}
        for sc in scenarios:
            rate = float(r[header_index[scenario_fields[sc]['rate']]]) * 100.0  # convert to %
            def safe_float(col_name):
                idx = header_index.get(col_name)
                if idx is None:
                    return math.nan
                val = r[idx].strip()
                if val == '':
                    return math.nan
                try:
                    return float(val) * 100.0  # also %
                except ValueError:
                    return math.nan
            ci_low = safe_float(scenario_fields[sc]['ci_low'])
            ci_high = safe_float(scenario_fields[sc]['ci_high'])
            per_scenario[sc]['rate'].append(rate)
            per_scenario[sc]['ci_low'].append(ci_low)
            per_scenario[sc]['ci_high'].append(ci_high)
    return np.array(rot_values), scenarios, per_scenario


def plot(rot_values, scenarios, per_scenario, save: str | None, dpi: int, show_ci: bool, title: str, ylim_max: float):
    plt.rcParams['pdf.fonttype'] = 42
    plt.rcParams['ps.fonttype'] = 42
    plt.rcParams['font.sans-serif'] = ["DejaVu Sans", "Arial", "Liberation Sans"]
    plt.rcParams['text.usetex'] = False

    fig, ax = plt.subplots(figsize=(5,3))
    # Use a stable color palette (tab10) so each scenario has a distinct color
    cmap = plt.get_cmap('tab10')
    for idx, sc in enumerate(scenarios):
        color = cmap(idx % cmap.N)
        label = sc.replace('_', ' ')
        rates = per_scenario[sc]['rate']
        ci_low = per_scenario[sc]['ci_low']
        ci_high = per_scenario[sc]['ci_high']
        if show_ci and not all(math.isnan(v) for v in ci_low) and not all(math.isnan(v) for v in ci_high):
            # Convert stored absolute bounds to symmetric error bars around rate
            lower_err = []
            upper_err = []
            for r, lo, hi in zip(rates, ci_low, ci_high):
                if math.isnan(lo) or math.isnan(hi):
                    lower_err.append(0.0)
                    upper_err.append(0.0)
                else:
                    lower_err.append(max(0.0, r - lo))
                    upper_err.append(max(0.0, hi - r))
            ax.errorbar(
                rot_values, rates, yerr=[lower_err, upper_err], marker='o',
                linewidth=1.4, capsize=4, label=label, alpha=0.75,
                elinewidth=1.0, ecolor=color, color=color, markersize=2
            )
        else:
            ax.plot(rot_values, rates, marker='o', linewidth=1.8, label=label, alpha=0.95, color=color, markersize=3)

    ax.set_xlabel('Intruder Rate of Turn [deg/s]')
    ax.set_ylabel('Success Rate [%]')
    if not title:
        title = 'Success Rate vs Intruder Turn Rate'
    ax.set_title(title)
    ax.grid(True, linewidth=0.6)
    ax.legend()
    ax.set_xlim(min(rot_values), max(rot_values))
    ax.set_ylim(0, ylim_max)

    fig.tight_layout()
    if save:
        Path(save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Replot success rate from existing CSV with extended Y limit')
    parser.add_argument('--csv', required=True, help='Input CSV exported earlier')
    parser.add_argument('--save', type=str, default=None, help='Output plot path (png/pdf/svg)')
    parser.add_argument('--dpi', type=int, default=600, help='DPI when saving')
    parser.add_argument('--no-ci', action='store_true', help='Disable confidence interval drawing')
    parser.add_argument('--title', type=str, default='', help='Custom plot title')
    parser.add_argument('--ylim-max', type=float, default=110.0, help='Upper Y-axis limit (default 110)')
    args = parser.parse_args()

    rot_values, scenarios, per_scenario = parse_csv(args.csv)
    plot(rot_values, scenarios, per_scenario, args.save, args.dpi, show_ci=not args.no_ci, title=args.title, ylim_max=args.ylim_max)

if __name__ == '__main__':
    main()
