# Angular-obstacle-avoidance

##run 
python BearingRateGraph_cleaned.py

You can change setting in 

run_simulation():
    # Initialize ownship and target ship statuses
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0, rate_of_turn=0, position=[0, 0, 0], size=0.5)
    # Target ship (Ship A) with initial position and velocity
    ship = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=-180.0, rate_of_turn=0, position=[50, -1, 0], size=0.5)
    # Goal ship for navigation
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0, rate_of_turn=0, position=[50, 0, 0])
    time_steps = 5000
    delta_time = 0.01