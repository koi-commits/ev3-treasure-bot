from sim import SIM_HOLDER, Done, Until


class DriveBase:
    def __init__(self, left, right, wheel_diameter, axle_track):
        left.side, right.side = "L", "R"
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self._settings = (200, 750, 150, 750)

    def _sim(self):
        return SIM_HOLDER.sim

    def settings(self, straight_speed=None, straight_acceleration=None, turn_rate=None, turn_acceleration=None):
        # Real DriveBase.settings() supports partial keyword updates.
        if all(v is None for v in (straight_speed, straight_acceleration, turn_rate, turn_acceleration)):
            return self._settings
        old = self._settings
        self._settings = tuple(old[i] if v is None else v for i, v in
                               enumerate((straight_speed, straight_acceleration, turn_rate, turn_acceleration)))

    def drive(self, speed, turn_rate):
        self._sim().cmd_drive(speed, turn_rate)

    def stop(self):
        self._sim().cmd_stop("coast")

    def brake(self):
        self._sim().cmd_stop("brake")

    def hold(self):
        self._sim().cmd_stop("hold")

    def straight(self, distance, then=None, wait=True):
        s = self._sim()
        s.cmd_straight(distance, self._settings[0])
        return Until(s.maneuver_done) if wait else Done()

    def turn(self, angle, then=None, wait=True, absolute=False):
        s = self._sim()
        s.cmd_turn(angle, self._settings[2])
        return Until(s.maneuver_done) if wait else Done()

    def done(self):
        return self._sim().maneuver_done()

    def distance(self):
        return self._sim().odo_distance()

    def angle(self):
        return self._sim().odo_angle()

    def state(self):
        s = self._sim()
        return (s.odo_distance(), s.v, s.odo_angle(), s.w)

    def reset(self, distance=0, angle=0):
        self._sim().cmd_reset(distance, angle)

    def stalled(self):
        return self._sim().stalled
