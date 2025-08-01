import math

DEG_PER_RAD = 180.0 / math.pi

class PhysicsObject:
    def __init__(
        self,
        acceleration=0,
        angular_acceleration=0,
        angular_velocity=0,
        heading=0,
        label="",
        max_history_length=50,
        position_x=0,
        position_y=0,
        velocity_x=0,
        velocity_y=0
    ):
        self.acceleration = acceleration
        self.angular_acceleration = angular_acceleration
        self.angular_velocity = angular_velocity
        self.heading = heading
        self.label = label
        self.max_history_length = max_history_length
        self.position_x = position_x
        self.position_y = position_y
        self.velocity_x = velocity_x
        self.velocity_y = velocity_y
        self.distance_from_player = 100
        self.bearing_histories = []

    def update(self):
        self.angular_velocity += self.angular_acceleration
        self.angular_velocity *= 0.9
        self.increment_heading(self.angular_velocity)

        rad = (self.heading - 90) / DEG_PER_RAD
        self.velocity_x += self.acceleration * math.cos(rad)
        self.velocity_y += self.acceleration * math.sin(rad)
        self.velocity_x *= 0.9
        self.velocity_y *= 0.9
        self.position_x += self.velocity_x
        self.position_y += self.velocity_y

    def update_distance_from_player(self, player):
        self.distance_from_player = math.hypot(
            self.position_x - player.position_x,
            self.position_y - player.position_y
        )

    def get_size(self):
        return min(40, 1000 / (self.distance_from_player + 10))

    def get_opacity(self):
        return 400 / (self.distance_from_player + 400)

    def add_bearing(self, bearing):
        self.bearing_histories.insert(0, bearing)
        if len(self.bearing_histories) > self.max_history_length:
            self.bearing_histories.pop()

    def set_heading(self, heading):
        self.heading = heading % 360

    def set_heading_from_dx_dy(self, dx, dy, account_heading=False):
        rad_heading = (self.heading / DEG_PER_RAD) if account_heading else 0
        degree_value = (rad_heading - math.atan2(dx, dy)) * DEG_PER_RAD
        self.set_heading(degree_value)

    def increment_heading(self, increment):
        self.set_heading(self.heading + increment)