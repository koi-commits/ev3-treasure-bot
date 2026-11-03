class _Enum:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self._prefix + self.name

    __str__ = __repr__


class _Port(_Enum):
    _prefix = "Port."


class Port:
    A, B, C, D = (_Port(n) for n in "ABCD")
    S1, S2, S3, S4 = (_Port(n) for n in ("S1", "S2", "S3", "S4"))


class _Dir(_Enum):
    _prefix = "Direction."


class Direction:
    CLOCKWISE = _Dir("CLOCKWISE")
    COUNTERCLOCKWISE = _Dir("COUNTERCLOCKWISE")


class _Btn(_Enum):
    _prefix = "Button."


class Button:
    LEFT, RIGHT, UP, DOWN, CENTER = (_Btn(n) for n in ("LEFT", "RIGHT", "UP", "DOWN", "CENTER"))


class _Stop(_Enum):
    _prefix = "Stop."


class Stop:
    COAST, BRAKE, HOLD = (_Stop(n) for n in ("COAST", "BRAKE", "HOLD"))


class Color:
    def __init__(self, h=0, s=0, v=0, name=None):
        self.h, self.s, self.v, self._name = h, s, v, name

    def __repr__(self):
        return "Color." + self._name if self._name else "Color(h={}, s={}, v={})".format(self.h, self.s, self.v)

    __str__ = __repr__


for _n, _hsv in (("RED", (0, 100, 100)), ("ORANGE", (30, 100, 100)), ("YELLOW", (60, 100, 100)),
                 ("GREEN", (120, 100, 100)), ("BLUE", (240, 100, 100)), ("WHITE", (0, 0, 100)),
                 ("BLACK", (0, 0, 10)), ("NONE", (0, 0, 0)), ("MAGENTA", (300, 100, 100))):
    setattr(Color, _n, Color(*_hsv, name=_n))
