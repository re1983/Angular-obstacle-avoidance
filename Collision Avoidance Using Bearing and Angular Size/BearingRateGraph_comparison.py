"""
Unified runner for all scenarios using a single entry point.
This script does NOT modify your original files. It dynamically loads a scenario config
and reuses the existing simulation/plot implementation.
"""

import argparse
import importlib
import importlib.util
import sys
from pathlib import Path

# Ensure repository root is on sys.path so we can import original modules
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.append(str(REPO_ROOT))

# We will reuse the canonical implementation from the UAV_vs_UAV folder
# because it exposes generic functions that accept configs.
Underlying = importlib.import_module(
    'monte_carlo_simulation_UAV_vs_UAV.BearingRateGraph_comparison'
)


def load_scenario_config(scenario_name: str):
    """Load scenario config by name from this package's configs via file path.

    Expected names: UAV_vs_UAV, B737_vs_UAV, Cessna_vs_UAV, Diamond
    """
    cfg_path = (
        Path(__file__).resolve().parent
        / 'configs'
        / scenario_name
        / 'config.py'
    )
    if not cfg_path.exists():
        raise FileNotFoundError(f"Config not found for scenario '{scenario_name}': {cfg_path}")
    mod_name = f"paper_cfg_{scenario_name}"
    spec = importlib.util.spec_from_file_location(mod_name, cfg_path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)  # type: ignore[attr-defined]
    return module


def make_ship_configs_from_config(cfg):
    """Translate a scenario config module into ship dictionaries expected by the underlying runner."""
    own = {
        "name": "Ownship",
        "velocity": getattr(cfg, 'OWNSHIP_VELOCITY'),
        "acceleration": 0,
        "heading": 0.0,
        "rate_of_turn": 0,
        "position": getattr(cfg, 'OWNSHIP_INITIAL_POSITION'),
        "size": getattr(cfg, 'OWNSHIP_SIZE'),
        "max_rate_of_turn": getattr(cfg, 'OWNSHIP_MAX_RATE_OF_TURN'),
    }
    goal = {
        "name": "Goal",
        "velocity": 0.0,
        "acceleration": 0,
        "heading": 0,
        "rate_of_turn": 0,
        "position": getattr(cfg, 'GOAL_POSITION'),
        "size": 0.1,
        "max_rate_of_turn": [0, 0],
    }

    # For a single run demo, generate a Ship A that will lead to a collision around the path
    # We don't randomize here; reviewers can change the config or use their own scripts.
    # Use heading 180 and no turn by default; velocities and size from mid-range or fixed values.
    import numpy as _np
    vmin, vmax = getattr(cfg, 'SHIP_A_VELOCITY_RANGE')
    smin, smax = getattr(cfg, 'SHIP_A_SIZE_RANGE')
    ship = Underlying.create_collision_scenario_with_turning(
        ship_velocity=float(vmin + (vmax - vmin) * 0.5),
        ship_heading=180.0,
        ship_rate_of_turn=0.0,
        ship_size=float(smin + (smax - smin) * 0.5),
        collision_ratio=0.5,
    )
    return own, ship, goal


def run_single(cfg, show_plot=True):
    own, ship, goal = make_ship_configs_from_config(cfg)
    res = Underlying.run_single_simulation(
        use_absolute_bearings=getattr(cfg, 'USE_ABSOLUTE_BEARINGS', True),
        ownship_config=own,
        ship_config=ship,
        goal_config=goal,
        time_steps=getattr(cfg, 'MAX_TIME_STEPS'),
        delta_time=getattr(cfg, 'DELTA_TIME'),
        ALPHA_TRIG=getattr(cfg, 'ALPHA_NAV', 1.0),
    )
    if show_plot:
        Underlying.plot_results(res, delta_time=getattr(cfg, 'DELTA_TIME'), title_prefix="Unified - ")
    return res


def run_comparison(cfg, show_plot=True):
    own, ship, goal = make_ship_configs_from_config(cfg)
    # Underlying exposes a comparison util; optionally suppress plotting
    _orig_plot = Underlying.plot_results
    try:
        if not show_plot:
            Underlying.plot_results = lambda *args, **kwargs: None  # type: ignore[assignment]
        Underlying.run_comparison(
            ownship_config=own,
            ship_config=ship,
            goal_config=goal,
            time_steps=getattr(cfg, 'MAX_TIME_STEPS'),
            delta_time=getattr(cfg, 'DELTA_TIME'),
            ALPHA_TRIG=getattr(cfg, 'ALPHA_NAV', 1.0),
        )
    finally:
        Underlying.plot_results = _orig_plot


def main():
    ap = argparse.ArgumentParser(description='Unified collision-avoidance simulations')
    ap.add_argument('--scenario', default='UAV_vs_UAV',
                    choices=['UAV_vs_UAV', 'B737_vs_UAV', 'Cessna_vs_UAV', 'Diamond'],
                    help='Scenario selection (config-driven)')
    ap.add_argument('--single', action='store_true', help='Run a single absolute-bearing simulation demo')
    ap.add_argument('--no-plot', action='store_true', help='Suppress plots')
    args = ap.parse_args()

    cfg = load_scenario_config(args.scenario)

    # Absolute-only flow
    run_single(cfg, show_plot=not args.no_plot)


if __name__ == '__main__':
    main()
