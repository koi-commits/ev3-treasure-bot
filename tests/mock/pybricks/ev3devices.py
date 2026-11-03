from sim import SIM_HOLDER, Value


class Motor:
    def __init__(self, port, positive_direction=None, gears=None, reset_angle=True, profile=None):
        self.port = port
        self.side = None   # set by DriveBase

    def load(self):
        return SIM_HOLDER.sim.motor_load(self.side)

    def angle(self):
        return SIM_HOLDER.sim.motor_angle(self.side)

    def speed(self, window=100):
        return 0

    def stalled(self):
        return SIM_HOLDER.sim.stalled

    def run(self, speed):
        # Single-wheel test: enough to cover API calls, not wheel physics.
        self._test_speed = speed

    def brake(self):
        self._test_speed = 0

    def stop(self):
        SIM_HOLDER.sim.cmd_stop()

    def close(self):
        pass


class ColorSensor:
    def __init__(self, port):
        self.port = port

    def reflection(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("reflection"))

    def rgb(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("rgb"))

    def ambient(self):
        return Value(lambda: 2.0)

    def color(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("color"))

    def hsv(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("hsv"))

    def detectable_colors(self, colors=None):
        return colors
