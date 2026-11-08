from sim import SIM_HOLDER, Done


class _Buttons:
    def pressed(self):
        return SIM_HOLDER.sim.buttons_pressed()


class _Light:
    def on(self, c):
        SIM_HOLDER.sim.light = c

    def off(self):
        SIM_HOLDER.sim.light = None

    def blink(self, c, d):
        SIM_HOLDER.sim.light = c


class _Speaker:
    def beep(self, frequency=500, duration=100):
        SIM_HOLDER.sim.beeps.append((SIM_HOLDER.sim.t, frequency))
        return Done()

    def volume(self, v=None):
        return 50


class _Screen:
    width, height = 178, 128

    def clear(self):
        pass

    def print(self, *a, sep=" ", end="\n"):
        SIM_HOLDER.sim.screen.append(sep.join(str(x) for x in a))

    def draw_text(self, *a, **k):
        pass

    def set_font(self, f):
        pass


class _Battery:
    def voltage(self):
        return SIM_HOLDER.sim.battery_voltage()

    def current(self):
        return 150


class _System:
    def info(self):
        return {"name": "SimHamster", "host_connected_usb": True}

    def set_stop_button(self, b):
        pass


class EV3Brick:
    def __init__(self):
        self.buttons = _Buttons()
        self.light = _Light()
        self.speaker = _Speaker()
        self.screen = _Screen()
        self.battery = _Battery()
        self.system = _System()
