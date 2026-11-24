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
        import config
        self.port = port
        # 右传感器读仿真的右光斑；其余端口读左光斑（巡线用）
        self.which = 1 if port == getattr(config, "COLOR_SENSOR_RIGHT_PORT", None) else 0
        if self.which == 1 and getattr(SIM_HOLDER.sim, "right_sensor", None) is None:
            raise OSError("no device on " + str(port))

    def reflection(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("reflection", self.which))

    def rgb(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("rgb", self.which))

    def ambient(self):
        return Value(lambda: 2.0)

    def color(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("color", self.which))

    def hsv(self):
        return Value(lambda: SIM_HOLDER.sim.read_sensor("hsv", self.which))

    def detectable_colors(self, colors=None):
        return colors


class TouchSensor:
    """2026-11-17 硬件 v3：V 形推板顶点的触感。仿真里默认没人按。

    和 ColorSensor 一样返回 Value()：实机上这些读值可能是 awaitable，
    mock 必须同构，否则"忘了 await"在 PC 上根本测不出来。
    """

    def __init__(self, port):
        self.port = port

    def pressed(self):
        return Value(lambda: bool(getattr(SIM_HOLDER.sim, "touch_pressed", False)))

    def close(self):
        pass


class GyroSensor:
    """2026-11-17 硬件 v3：陀螺仪。

    仿真里读 sim.gyro_angle()（真实航向 + 偏置漂移 + 读数噪声），而不是里程计航向 ——
    否则 "陀螺漂移" 这类场景根本没有意义。reset_angle 用不含噪声的 gyro_raw() 记账。
    """

    def __init__(self, port):
        self.port = port
        self._offset = 0.0

    def angle(self):
        return Value(lambda: SIM_HOLDER.sim.gyro_angle() - self._offset)

    def speed(self):
        return Value(lambda: SIM_HOLDER.sim.gyro_rate())

    def reset_angle(self, angle=0):
        self._offset = SIM_HOLDER.sim.gyro_raw() - angle

    def close(self):
        pass
