"""
Unified Monte Carlo launcher that delegates to the original scenario-specific runners
without modifying existing files. Absolute-bearing only (as in configs).
"""

import argparse
import importlib
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

SCENARIO_TO_RUNNER = {
    'UAV_vs_UAV': 'monte_carlo_simulation_UAV_vs_UAV.monte_carlo_runner_multithread',
    'B737_vs_UAV': 'monte_carlo_simulation_B737_vs_UAV.monte_carlo_runner_multithread',
    'Cessna_vs_UAV': 'monte_carlo_simulation_Cessna_vs_UAV.monte_carlo_runner_multithread',
    'Diamond': 'MCS_diamond.monte_carlo_runner_multithread',
}

def main():
    ap = argparse.ArgumentParser(description='Unified Monte Carlo batch runner (absolute only)')
    ap.add_argument('--scenario', default='UAV_vs_UAV',
                    choices=list(SCENARIO_TO_RUNNER.keys()),
                    help='Scenario selection')
    ap.add_argument('--workers', type=int, default=None, help='Number of parallel workers (default: CPUs-1)')
    ap.add_argument('--serial', action='store_true', help='Run in serial mode instead of parallel')
    ap.add_argument('--count', type=int, default=None, help='Override number of simulations for this run')
    args = ap.parse_args()

    # Ensure scenario folder is first on sys.path and set CWD so outputs land under that folder
    runner_mod_path = SCENARIO_TO_RUNNER[args.scenario]
    scenario_pkg = runner_mod_path.split('.')[0]
    scenario_dir = REPO_ROOT / scenario_pkg
    if str(scenario_dir) not in sys.path:
        sys.path.insert(0, str(scenario_dir))
    os.chdir(str(scenario_dir))

    mod = importlib.import_module(runner_mod_path)
    Runner = getattr(mod, 'MonteCarloRunner')
    runner = Runner()

    if args.serial:
        runner.run_monte_carlo_batch(total=args.count)
    else:
        runner.run_monte_carlo_parallel(workers=args.workers, total=args.count)

if __name__ == '__main__':
    main()
