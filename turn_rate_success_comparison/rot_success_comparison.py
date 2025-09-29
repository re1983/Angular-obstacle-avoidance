"""
Compare success rate vs fixed intruder rate-of-turn across scenarios.

Scenarios included:
  - B737_vs_UAV
  - Cessna_vs_UAV
  - UAV_vs_UAV

For each scenario and each specified rate-of-turn (deg/s), we run N Monte Carlo
simulations with Ship A's rate-of-turn range forced to [rot, rot] (deterministic ROT)
while leaving other randomized parameters intact. We then compute success rate
 (successful / total) ignoring collision/timeout counts separately.

Outputs:
  - Matplotlib figure with three curves (success rate vs ROT)
  - Optional CSV with detailed counts
  - Optional PNG/PDF/SVG export of the plot

Usage example:
    python3 rot_success_comparison.py --rot-start 1 --rot-stop 10 --rot-step 1 \
        --runs 200 --save-plot rot_success.png --csv rot_success.csv

NOTE: This script imports each scenario's multithread Monte Carlo runner, but executes
its single-run logic synchronously to keep reproducibility and control. For speed, you
may raise --runs or parallelize in future.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import shutil
import uuid
from contextlib import contextmanager
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))


SCENARIO_MODULES = {
    'B737_vs_UAV': 'monte_carlo_simulation_B737_vs_UAV.monte_carlo_runner_multithread',
    'Cessna_vs_UAV': 'monte_carlo_simulation_Cessna_vs_UAV.monte_carlo_runner_multithread',
    'UAV_vs_UAV': 'monte_carlo_simulation_UAV_vs_UAV.monte_carlo_runner_multithread',
}


@dataclass
class SweepResult:
    rot: float
    total: int
    successful: int
    collision: int
    timeout: int
    ci_low: float | None = None  # success rate lower (0-1)
    ci_high: float | None = None # success rate upper (0-1)

    @property
    def success_rate(self) -> float:
        return self.successful / self.total if self.total else 0.0


@contextmanager
def scenario_import_context(module_path: str):
    """Temporarily adjust sys.path and CWD so that intra-scenario relative imports
    (e.g., `from BearingRateGraph_comparison import ...`) resolve to the scenario's
    local file instead of a root-level module with the same name.
    """
    scenario_dir_name = module_path.split('.')[0]
    scenario_dir = REPO_ROOT / scenario_dir_name
    prev_cwd = Path.cwd()
    # Insert scenario dir at front of sys.path for import precedence
    sys.path.insert(0, str(scenario_dir))
    try:
        os.chdir(scenario_dir)
        # Remove cached root-level BearingRateGraph_comparison if already imported
        sys.modules.pop('BearingRateGraph_comparison', None)
        yield
    finally:
        # Restore state
        os.chdir(prev_cwd)
        try:
            sys.path.remove(str(scenario_dir))
        except ValueError:
            pass


def load_scenario_runner(module_path: str):
    with scenario_import_context(module_path):
        return importlib.import_module(module_path)


def run_single_with_fixed_rot(runner_cls, rot_deg: float, random_seed=None, suppress_artifacts: bool = False):
    """Instantiate a runner and execute one simulation with Ship A ROT forced to rot_deg.

    We override the random generator for rate_of_turn only; other random draws follow original logic.
    """
    runner = runner_cls()
    # Monkey patch generate_random_ship_parameters to enforce specific rate_of_turn
    orig_gen = runner.generate_random_ship_parameters

    def patched_gen():
        p = orig_gen()
        p['rate_of_turn'] = rot_deg  # force constant ROT
        # Constrain range attributes if referenced elsewhere (not required for single run, but explicit)
        return p

    runner.generate_random_ship_parameters = patched_gen  # type: ignore

    if suppress_artifacts:
        # Redirect results_dir to a temp unique folder to avoid collisions
        temp_dir = Path('tmp_rot_no_artifacts') / f"{uuid.uuid4().hex}"
        # Create minimal structure expected by code
        for sub in [temp_dir, temp_dir / 'successful', temp_dir / 'collision', temp_dir / 'timeout']:
            sub.mkdir(parents=True, exist_ok=True)
        try:
            runner.results_dir = temp_dir  # type: ignore[attr-defined]
        except Exception:
            pass
        # Monkey patch save methods / helpers to no-op to skip file writes
        if hasattr(runner, 'save_individual_result'):
            runner.save_individual_result = lambda *a, **k: None  # type: ignore
        # Patch module-level helper if exists in its module
        mod_ref = sys.modules.get(runner_cls.__module__)
        if mod_ref is not None and '_save_individual_result' in getattr(mod_ref, '__dict__', {}):
            setattr(mod_ref, '_save_individual_result', lambda *a, **k: None)

    sim_id = 1  # not used meaningfully for single run
    data = runner.run_single_monte_carlo(sim_id)

    if suppress_artifacts:
        # Best-effort cleanup of temp directory; ignore errors
        try:
            shutil.rmtree(runner.results_dir, ignore_errors=True)  # type: ignore
        except Exception:
            pass
    success_type = data['result']['success']
    return success_type


def sweep_rot_for_scenario(module_path: str, rot_values: np.ndarray, runs_per_rot: int, seed: int | None, suppress_artifacts: bool=False) -> List[SweepResult]:
    mod = load_scenario_runner(module_path)
    runner_cls = getattr(mod, 'MonteCarloRunner')
    results: List[SweepResult] = []

    base_rng = np.random.default_rng(seed)

    for rot in rot_values:
        successful = collision = timeout = 0
        # Derive a stable sub-seed for this ROT
        rot_seed = None
        if seed is not None:
            rot_seed = base_rng.integers(0, 2**32 - 1)
        per_rot_rng = np.random.default_rng(rot_seed)
        for run_idx in range(runs_per_rot):
            if seed is not None:
                # Deterministic per run seed derived from rot_seed and run index
                run_seed = (int(rot_seed) + run_idx * 9973) % (2**32 - 1)  # 9973 is a prime multiplier
                np.random.seed(run_seed)
            status = run_single_with_fixed_rot(runner_cls, rot, suppress_artifacts=True if suppress_artifacts else False)
            if status == 'successful':
                successful += 1
            elif status == 'collision':
                collision += 1
            elif status == 'timeout':
                timeout += 1
        results.append(SweepResult(rot=rot, total=runs_per_rot, successful=successful, collision=collision, timeout=timeout))
    return results


# ---------- Parallel version (per simulation) ----------
def _simulate_one(args_tuple):
    module_path, rot, run_seed, fast_no_save, suppress_artifacts = args_tuple
    # Import in isolated context to ensure correct scenario file resolutions
    with scenario_import_context(module_path):
        mod = importlib.import_module(module_path)
        runner_cls = getattr(mod, 'MonteCarloRunner')
        # Optional patch to suppress saving artifacts for speed
        if fast_no_save:
            # Patch _save_individual_result if present in module namespace
            if '_save_individual_result' in mod.__dict__:
                mod._save_individual_result = lambda *a, **k: None  # type: ignore
        if run_seed is not None:
            np.random.seed(run_seed)
    status = run_single_with_fixed_rot(runner_cls, rot, suppress_artifacts=suppress_artifacts or fast_no_save)
    return status


def sweep_rot_parallel(module_path: str, rot_values: np.ndarray, runs_per_rot: int, seed: int | None, workers: int, fast_no_save: bool, suppress_artifacts: bool) -> List[SweepResult]:
    import multiprocessing as mp
    results: List[SweepResult] = []
    base_rng = np.random.default_rng(seed)
    for rot in rot_values:
        # Prepare work items with deterministic seeds
        tasks = []
        for run_idx in range(runs_per_rot):
            run_seed = None
            if seed is not None:
                run_seed = base_rng.integers(0, 2**32 - 1)
            tasks.append((module_path, float(rot), run_seed, fast_no_save, suppress_artifacts))
        successful = collision = timeout = 0
        with mp.Pool(processes=workers) as pool:
            for status in pool.imap_unordered(_simulate_one, tasks):
                if status == 'successful':
                    successful += 1
                elif status == 'collision':
                    collision += 1
                elif status == 'timeout':
                    timeout += 1
        results.append(SweepResult(rot=float(rot), total=runs_per_rot, successful=successful, collision=collision, timeout=timeout))
    return results


def compute_wilson_ci(successes: int, n: int, confidence: float) -> tuple[float, float]:
    if n == 0:
        return 0.0, 0.0
    from math import sqrt
    z = 0.0
    # Approximate z via inverse error for common confidence levels
    # Provide standard 90/95/99 else fallback to 95
    if abs(confidence - 0.90) < 1e-6:
        z = 1.6448536269514722
    elif abs(confidence - 0.95) < 1e-6:
        z = 1.959963984540054
    elif abs(confidence - 0.99) < 1e-6:
        z = 2.5758293035489004
    else:
        z = 1.959963984540054
    p_hat = successes / n
    denom = 1 + (z**2)/n
    center = (p_hat + (z**2)/(2*n)) / denom
    half = z * sqrt(p_hat*(1-p_hat)/n + (z**2)/(4*n*n)) / denom
    low = max(0.0, center - half)
    high = min(1.0, center + half)
    return low, high


def plot_results(all_results: Dict[str, List[SweepResult]], save_plot: str | None, dpi: int, show_ci: bool):
    plt.rcParams['pdf.fonttype'] = 42
    plt.rcParams['ps.fonttype'] = 42
    plt.rcParams['font.sans-serif'] = ["DejaVu Sans", "Arial", "Liberation Sans"]
    plt.rcParams['text.usetex'] = False

    fig, ax = plt.subplots(figsize=(8, 5))

    for scenario, res_list in all_results.items():
        display_label = scenario.replace('_', ' ')
        rots = [r.rot for r in res_list]
        rates = [r.success_rate * 100.0 for r in res_list]
        if show_ci and any(r.ci_low is not None for r in res_list):
            lows = []
            highs = []
            for r in res_list:
                if r.ci_low is None or r.ci_high is None:
                    lows.append(0.0)
                    highs.append(0.0)
                else:
                    low_err = (r.success_rate - r.ci_low) * 100.0
                    high_err = (r.ci_high - r.success_rate) * 100.0
                    if low_err < 0:
                        low_err = 0.0
                    if high_err < 0:
                        high_err = 0.0
                    lows.append(low_err)
                    highs.append(high_err)
            ax.errorbar(rots, rates, yerr=[lows, highs], marker='o', linewidth=1.4, capsize=4, label=display_label)
        else:
            ax.plot(rots, rates, marker='o', linewidth=1.8, label=display_label)

    ax.set_xlabel('Intruder Rate of Turn [deg/s]')
    ax.set_ylabel('Success Rate [%]')
    ax.set_title('Success Rate vs Intruder Turn Rate')
    ax.grid(True, linewidth=0.6)
    ax.legend()
    # Use union of all ROT values for limits
    all_rots = sorted({r.rot for res in all_results.values() for r in res})
    ax.set_xlim(min(all_rots), max(all_rots))
    ax.set_ylim(0, 100)

    fig.tight_layout()
    if save_plot:
        Path(save_plot).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_plot, dpi=dpi, bbox_inches='tight')
        plt.close(fig)
    else:
        plt.show()


def write_csv(all_results: Dict[str, List[SweepResult]], csv_path: str):
    import csv
    Path(csv_path).parent.mkdir(parents=True, exist_ok=True)
    rot_values = sorted({r.rot for res in all_results.values() for r in res})
    scenarios = list(all_results.keys())
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ['rot_deg']
        for sc in scenarios:
            header.extend([f'{sc}_success_rate', f'{sc}_ci_low', f'{sc}_ci_high', f'{sc}_successful', f'{sc}_collision', f'{sc}_timeout', f'{sc}_total'])
        writer.writerow(header)
        for rot in rot_values:
            row = [rot]
            for sc in scenarios:
                rec = next(r for r in all_results[sc] if abs(r.rot - rot) < 1e-9)
                row.extend([
                    f'{rec.success_rate:.4f}',
                    f'{rec.ci_low:.4f}' if rec.ci_low is not None else '',
                    f'{rec.ci_high:.4f}' if rec.ci_high is not None else '',
                    rec.successful, rec.collision, rec.timeout, rec.total
                ])
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description='Success rate vs intruder rate-of-turn comparison across scenarios')
    parser.add_argument('--rot-start', type=float, default=1.0, help='Start ROT (deg/s) inclusive')
    parser.add_argument('--rot-stop', type=float, default=10.0, help='Stop ROT (deg/s) inclusive')
    parser.add_argument('--rot-step', type=float, default=1.0, help='ROT step (deg/s)')
    parser.add_argument('--runs', type=int, default=50, help='Monte Carlo runs per ROT per scenario')
    parser.add_argument('--seed', type=int, default=42, help='Base random seed (set None for random)')
    parser.add_argument('--save-plot', type=str, default=None, help='Path to save plot (PNG/PDF/SVG)')
    parser.add_argument('--csv', type=str, default=None, help='Optional CSV output path')
    parser.add_argument('--dpi', type=int, default=600, help='DPI for saved plot')
    parser.add_argument('--ci', action='store_true', help='Show 95% confidence interval (Wilson) as error bars')
    parser.add_argument('--confidence', type=float, default=0.95, help='Confidence level for CI (default 0.95)')
    parser.add_argument('--parallel-workers', type=int, default=1, help='Parallel worker processes (per scenario). >1 enables parallel mode.')
    parser.add_argument('--fast-no-save', action='store_true', help='Speed up by suppressing per-simulation file output (monkey patched)')
    parser.add_argument('--no-artifacts', action='store_true', help='Do not keep per-simulation results directories (they are deleted after each run)')
    args = parser.parse_args()

    if args.rot_step <= 0:
        raise ValueError('--rot-step must be > 0')

    rot_values = np.arange(args.rot_start, args.rot_stop + 1e-9, args.rot_step)

    all_results: Dict[str, List[SweepResult]] = {}
    for scenario, module_path in SCENARIO_MODULES.items():
        print(f"\n=== Scenario: {scenario} ===")
        if args.parallel_workers > 1:
            res_list = sweep_rot_parallel(module_path, rot_values, args.runs, args.seed, args.parallel_workers, args.fast_no_save, args.no_artifacts)
        else:
            res_list = sweep_rot_for_scenario(module_path, rot_values, args.runs, args.seed, suppress_artifacts=args.no_artifacts or args.fast_no_save)
        # Compute confidence intervals if requested
        if args.ci:
            for rec in res_list:
                low, high = compute_wilson_ci(rec.successful, rec.total, args.confidence)
                rec.ci_low = low
                rec.ci_high = high
        all_results[scenario] = res_list
        for rec in res_list:
            ci_txt = ''
            if args.ci:
                ci_txt = f" CI[{args.confidence*100:.0f}%]: {rec.ci_low*100:.1f}-{rec.ci_high*100:.1f}%"
            print(f"ROT {rec.rot:.2f} deg/s -> success {rec.successful}/{rec.total} ({rec.success_rate*100:.1f}%), collisions {rec.collision}, timeout {rec.timeout}{ci_txt}")

    plot_results(all_results, args.save_plot, args.dpi, show_ci=args.ci)

    if args.csv:
        write_csv(all_results, args.csv)
        print(f"CSV written: {args.csv}")


if __name__ == '__main__':
    main()
