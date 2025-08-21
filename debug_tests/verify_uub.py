import numpy as np
from math import radians, sin, cos, tan

# Simple imports from comparison file by reusing logic inline to avoid circular imports
# If needed, you can import from BearingRateGraph_comparison, but we copy minimal pieces here for isolation.

ALPHA_NAV = 1.0  # deg

class ShipStatus:
    def __init__(self, name, velocity, acceleration, heading, rate_of_turn, position, size=0.5, max_rate_of_turn=(12.0, 12.0)):
        self.name = name
        self.velocity = float(velocity)
        self.acceleration = float(acceleration)
        self.heading = float(heading)
        self.rate_of_turn = float(rate_of_turn)
        self.position = np.array(position, dtype=float)
        self.size = float(size)
        self.max_rate_of_turn = list(max_rate_of_turn)

    def update(self, dt=0.01):
        self.heading += self.rate_of_turn * dt
        self.position += self.velocity * dt * np.array([
            np.cos(np.radians(self.heading)),
            np.sin(np.radians(self.heading)),
            0.0,
        ])
        self.velocity += self.acceleration


def get_distance_3d(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))


def get_bearing(ship1, ship2):
    d = ship2.position - ship1.position
    theta = np.degrees(np.arctan2(d[1], d[0]))
    rb = (theta - ship1.heading) % 360
    if rb > 180:
        rb -= 360
    return rb


def get_absolute_bearing(ship1, ship2):
    d = ship2.position - ship1.position
    return (np.degrees(np.arctan2(d[1], d[0])) % 360)


def get_angular_diameter(ship1, ship2):
    R = get_distance_3d(ship1.position, ship2.position)
    if R <= 1e-9:
        return 180.0
    return np.degrees(2 * np.arctan(ship2.size / (2 * R)))


def angle_diff_deg(a1, a2):
    d = (a2 - a1) % 360
    if d > 180:
        d -= 360
    return d


def ctrl_absolute(abs_bearings, abs_rates, angular_sizes, own, goal, target, dt):
    velocity = own.velocity
    u = own.rate_of_turn
    umax = own.max_rate_of_turn[0]
    beta = get_bearing(own, target)
    gain = angular_sizes[-1] ** 2

    if len(abs_rates) >= 1:
        if abs(abs_rates[-1] * dt) <= angular_sizes[-1]:
            rounded = np.round(abs_rates[-1], 5)
            if abs(rounded) <= 1e-5:
                u = -umax if beta < 0 else umax
            else:
                u = -np.sign(abs_rates[-1]) * gain if abs(beta) < 90 else np.sign(abs_rates[-1]) * gain
        if angular_sizes[-1] < ALPHA_NAV:
            to_goal = get_bearing(own, goal)
            u = to_goal
            d = get_distance_3d(own.position, goal.position)
            velocity = d if d < 1 else 1.0
        u = float(np.clip(u, -umax, umax))
    return u, velocity


def ctrl_relative(bearings, rates, angular_sizes, own, goal, target, dt):
    velocity = own.velocity
    u = own.rate_of_turn
    umax = own.max_rate_of_turn[0]
    beta = get_bearing(own, target)
    gain = angular_sizes[-1] ** 2

    if len(rates) >= 1:
        if abs(rates[-1] * dt) <= angular_sizes[-1]:
            rounded = np.round(rates[-1], 5)
            if abs(rounded) <= 1e-5:
                u = -umax if beta < 0 else umax
            else:
                u = -np.sign(rates[-1]) * gain if abs(beta) < 90 else np.sign(rates[-1]) * gain
        if angular_sizes[-1] < ALPHA_NAV:
            to_goal = get_bearing(own, goal)
            u = to_goal
            d = get_distance_3d(own.position, goal.position)
            velocity = d if d < 1 else 1.0
        u = float(np.clip(u, -umax, umax))
    return u, velocity


def simulate(use_absolute=True, steps=5000, dt=0.01, size=0.5):
    own = ShipStatus("Own", velocity=1.0, acceleration=0.0, heading=90.0, rate_of_turn=0.0, position=[0, 0, 0], size=size)
    tgt = ShipStatus("Tgt", velocity=2.0, acceleration=0.0, heading=90.0, rate_of_turn=1.0, position=[15, -15, 0], size=size)
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0.0, heading=0.0, rate_of_turn=0.0, position=[0, 50, 0], size=size)

    bearings = []
    abs_bearings = []
    rates = []
    abs_rates = []
    angular_sizes = []
    distances = []

    for _ in range(steps):
        b = get_bearing(own, tgt)
        ab = get_absolute_bearing(own, tgt)
        a = get_angular_diameter(own, tgt)

        bearings.append(b)
        abs_bearings.append(ab)
        angular_sizes.append(a)
        distances.append(get_distance_3d(own.position, tgt.position))

        if use_absolute:
            u, v = ctrl_absolute(abs_bearings, abs_rates, angular_sizes, own, goal, tgt, dt)
        else:
            u, v = ctrl_relative(bearings, rates, angular_sizes, own, goal, tgt, dt)

        own.rate_of_turn, own.velocity = u, v
        own.update(dt)
        tgt.update(dt)

        # update rates
        ab2 = get_absolute_bearing(own, tgt)
        b2 = get_bearing(own, tgt)
        abs_rates.append(angle_diff_deg(ab, ab2) / dt)
        rates.append(angle_diff_deg(b, b2) / dt)

    return {
        'distances': np.array(distances),
        'angular_sizes': np.array(angular_sizes),
        'bearings': np.array(bearings),
        'abs_bearings': np.array(abs_bearings),
        'rates': np.array(rates),
        'abs_rates': np.array(abs_rates),
    }


def verify_case(R_SAFE=1.0):
    res_abs = simulate(True)
    res_rel = simulate(False)

    def summarize(name, res):
        dmin = float(np.min(res['distances']))
        amax = float(np.max(res['angular_sizes']))
        cbd_hits = float(np.mean((np.abs(res['abs_rates' if name=='ABS' else 'rates']) * 0.01) <= res['angular_sizes']))
        print(f"[{name}] min distance = {dmin:.3f} m, max alpha = {amax:.3f} deg, CBDR-threshold hit ratio ~ {cbd_hits:.2f}")
        ok = dmin >= R_SAFE
        return ok, dmin

    ok_abs, dmin_abs = summarize('ABS', res_abs)
    ok_rel, dmin_rel = summarize('REL', res_rel)

    all_ok = ok_abs and ok_rel
    print("Result:", "SAFE+UUB observed (practical)" if all_ok else "Potential violation observed; adjust gains or R_SAFE")


if __name__ == "__main__":
    # default safe distance equals to 1m (twice radius 0.5 as in examples)
    verify_case(R_SAFE=2.0)
