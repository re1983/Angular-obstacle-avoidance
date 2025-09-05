"""
Configuration file for Monte Carlo simulation parameters.
All simulation parameters can be adjusted here.
"""

import numpy as np

# =============================================================================
# Simulation Parameters
# =============================================================================

# Time parameters (from BearingRateGraph_cleaned.py defaults)
DELTA_TIME = 0.01  # seconds
MAX_TIME_STEPS = 75000  # maximum simulation steps (increased for longer simulations)
MAX_SIMULATION_TIME = MAX_TIME_STEPS * DELTA_TIME  # seconds

# Monte Carlo parameters
NUM_SIMULATIONS = 100  # number of Monte Carlo runs 
USE_ABSOLUTE_BEARINGS = True  # True for absolute, False for relative

# Individual trajectory saving options
SAVE_INDIVIDUAL_TRAJECTORIES = False  # Whether to save individual trajectory plots
SAVE_INDIVIDUAL_PARAMETERS = True   # Whether to save individual simulation parameters
SHOW_INDIVIDUAL_PLOTS = False  # Whether to display individual plots (False = save only)

# =============================================================================
# Ship Parameters (Based on BearingRateGraph_cleaned.py defaults)
# =============================================================================

# Ownship (fixed parameters - from BearingRateGraph_cleaned.py)
OWNSHIP_INITIAL_POSITION = [0.0, 0.0, 0.0]  # [North, East, Down]
OWNSHIP_VELOCITY = 15.0  # m/s (constant)
OWNSHIP_SIZE = 2.0  # m
OWNSHIP_MAX_RATE_OF_TURN = [20.0, 20.0]  # deg/s (original default)

# Goal parameters (from BearingRateGraph_cleaned.py)
GOAL_POSITION = [1500.0, 0.0, 0.0]  # [North, East, Down]

# Ship A (randomized parameters - ranges)
SHIP_A_SIZE_RANGE = [28, 42]  # [min, max] m
SHIP_A_VELOCITY_RANGE = [170, 180]  # [min, max] m/s
SHIP_A_HEADING_RANGE = [-180.0, 180.0]  # [min, max] degrees
SHIP_A_MAX_RATE_OF_TURN = [0.0, 0.0]  # deg/s (fixed) - straight line motion
SHIP_A_VELOCITY_LIMIT = [0.5, 10.0]  # [min, max] m/s (original default, not used)

# =============================================================================
# Collision Geometry Parameters
# =============================================================================

# Collision zone along Ownship's path (as percentage of total distance)
COLLISION_ZONE_START_RATIO = 0.4  # 40% of the way to goal
COLLISION_ZONE_END_RATIO = 0.6    # 60% of the way to goal

# Exclusion zones (distances from start/goal to avoid collisions)
START_EXCLUSION_DISTANCE = 5.0  # m from Ownship start position
GOAL_EXCLUSION_DISTANCE = 5.0   # m from Goal position

# Ship A spawn area constraints
SHIP_A_MIN_SPAWN_DISTANCE = 10.0  # m minimum distance from Ownship path
SHIP_A_MAX_SPAWN_DISTANCE = 100.0  # m maximum distance from Ownship path

# =============================================================================
# Navigation and Safety Parameters
# =============================================================================

# Navigation threshold constant (degrees)
ALPHA_NAV = 0.5 # No collision threat below this angular diameter

# Collision detection thresholds
MIN_SAFE_DISTANCE = 2.0  # m minimum safe distance between ships
COLLISION_THRESHOLD = 1.0  # m distance below which collision is considered

# =============================================================================
# Analysis Parameters
# =============================================================================

# Success criteria
SUCCESS_MIN_DISTANCE_THRESHOLD = MIN_SAFE_DISTANCE  # m
SUCCESS_GOAL_REACH_THRESHOLD = 2.0  # m distance from goal to consider "reached"

# Visualization parameters
PLOT_TRAJECTORY_SAMPLES = 10  # number of sample trajectories to plot
ARROW_INTERVAL_TIME = 10.0  # s interval for direction arrows
ARROW_LENGTH = 5.0  # m length of direction arrows

# Statistical analysis
CONFIDENCE_LEVEL = 0.95  # for confidence intervals
PERFORMANCE_METRICS = [
    'collision_avoided',
    'goal_reached', 
    'min_distance',
    'time_to_goal',
    'path_efficiency'
]

# =============================================================================
# Random Seed (set to None for truly random results)
# =============================================================================
RANDOM_SEED = None  # Set to integer for reproducible results
# RANDOM_SEED = 42  # Uncomment for reproducible results
