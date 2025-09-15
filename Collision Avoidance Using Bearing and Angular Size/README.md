# Collision Avoidance Using Bearing and Angular Size

A clean, submission-ready package to run and reproduce all Monte Carlo experiments and single-case backtests from your paper, without modifying the original repository files.

This package provides:

- A single, unified entry point `BearingRateGraph_comparison.py` that runs any scenario using a scenario-specific `config.py`.
- Separate scenario folders that mirror the existing ones but only contain a `config.py` to select parameters.
- A preserved backtesting flow from existing results text files via `one_monte_carlo_simulation/simple_backtest.py` (untouched).

## Folder Layout

```text
Collision Avoidance Using Bearing and Angular Size/
  README.md
  BearingRateGraph_comparison.py   # unified runner (imports your existing simulation code)
  configs/
    B737_vs_UAV/config.py
    Cessna_vs_UAV/config.py
    UAV_vs_UAV/config.py
    Diamond/config.py
```

Notes:

- We do not duplicate your simulation code. The unified runner imports the original modules from the repository and only switches configurations.
- Your existing scenario folders remain unchanged.


## Quick Start

1) Choose a scenario config and run the unified script.

Examples:

```bash
# UAV vs UAV (default)
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario UAV_vs_UAV

# B737 vs UAV
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario B737_vs_UAV

# Cessna vs UAV
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario Cessna_vs_UAV

# Diamond scenario
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario Diamond
```

Optional flags:

- `--single` to run one absolute-bearing simulation demo using the current config and show plots
- `--no-plot` to suppress plots

### Monte Carlo batch (absolute only)

Run the scenario’s existing multithreaded Monte Carlo runner via a unified wrapper. Outputs and report .txt files are written under the original scenario folder (for example: `monte_carlo_simulation_UAV_vs_UAV/results/...`).

```bash
# Parallel (default, detects CPUs-1)
python3 "Collision Avoidance Using Bearing and Angular Size/run_monte_carlo.py" --scenario UAV_vs_UAV

# Serial mode, override count
python3 "Collision Avoidance Using Bearing and Angular Size/run_monte_carlo.py" --scenario UAV_vs_UAV --serial --count 50

# Control workers
python3 "Collision Avoidance Using Bearing and Angular Size/run_monte_carlo.py" --scenario B737_vs_UAV --workers 8 --count 500
```

This will produce paths like:

```text
monte_carlo_simulation_UAV_vs_UAV/results/results_YYYYMMDD_HHMMSS/report_YYYYMMDD_HHMMSS.txt
```

Typical output structure:

```text
<scenario>/results/results_YYYYMMDD_HHMMSS/
  report_YYYYMMDD_HHMMSS.txt           # roll-up statistics
  simulation_results_YYYYMMDD_HHMMSS.npz
  successful/                           # per-simulation .txt for successful cases
    00001.txt
    00002.txt
  collision/
    00042.txt                           # per-simulation .txt for collision cases
  timeout/
    00123.txt                           # per-simulation .txt for timeouts
```

Each per-simulation .txt includes:

- Ship A Parameters (velocity, heading, size, rate_of_turn, collision_ratio, initial_position)
- Collision Prediction (point, time, motion_type)
- Actual Collision Location (min-distance info), time at min distance
- Ownship/Goal configuration
- Simulation parameters (delta_time, max_time_steps, use_absolute_bearings, alpha_nav)
- Simulation results (collision_time, arrival_time, timeout, simulation_time, min_distance, max_angular_size)

### Backtesting (unchanged)

Your existing backtest tool remains intact. Example:

```bash
python3 one_monte_carlo_simulation/simple_backtest.py results/collision/00092.txt
```

More backtest examples:

```bash
# Backtest a file from a Monte Carlo batch you just ran (successful case)
python3 one_monte_carlo_simulation/simple_backtest.py \
  monte_carlo_simulation_UAV_vs_UAV/results/results_YYYYMMDD_HHMMSS/successful/00001.txt

# Backtest a collision case
python3 one_monte_carlo_simulation/simple_backtest.py \
  monte_carlo_simulation_UAV_vs_UAV/results/results_YYYYMMDD_HHMMSS/collision/00042.txt
```

Tip: backtest reads all config values from the .txt file; it does not need a scenario config.

## How It Works

- This runner dynamically loads the chosen scenario's `config.py` and passes values into the existing `monte_carlo_simulation_*` code paths. It reuses the original `BearingRateGraph_comparison.py` from `monte_carlo_simulation_UAV_vs_UAV` as the canonical plot/simulation utilities, which are compatible across scenarios.
- No existing files are edited.

## Reproducibility

- You can set a fixed `RANDOM_SEED` in each scenario config to reproduce experiments.

Quick example (edit the scenario’s original `config.py`):

```python
RANDOM_SEED = 42
```

Then rerun Monte Carlo for deterministic sampling across workers.

## Output

- Results and plots are written according to the behavior of the underlying scenario code. This wrapper does not move or rename your original output directories.

## Requirements

- Python 3.10+
- numpy, matplotlib

If your environment matches the repository that produced the original results, you should be ready to run directly.

## Support

If you need variant configs or extra scenarios, add another folder and a `config.py` under `configs/`, then point `--scenario` to that name.

### Extra quick demos (absolute-only)

```bash
# Run demo without plots for each scenario
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario UAV_vs_UAV --no-plot
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario B737_vs_UAV --no-plot
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario Cessna_vs_UAV --no-plot
python3 "Collision Avoidance Using Bearing and Angular Size/BearingRateGraph_comparison.py" --scenario Diamond --no-plot

# Small Monte Carlo smoke tests
python3 "Collision Avoidance Using Bearing and Angular Size/run_monte_carlo.py" --scenario UAV_vs_UAV --serial --count 2
python3 "Collision Avoidance Using Bearing and Angular Size/run_monte_carlo.py" --scenario B737_vs_UAV --workers 4 --count 50
```
