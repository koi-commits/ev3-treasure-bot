r"""
sensing.py — 颜色传感器读数 -> (是否白边, 色标标签)。

关键设计：**每个 tick 只读一种传感器模式**。
EV3 颜色传感器的 reflection / rgb / color / hsv 对应不同的硬件模式，
在同一循环里轮流调用会反复切模式。11-03 的 color_probe 每轮调 hsv+rgb+color，
读数 20 s 纹丝不动 —— 与"模式切换导致读到旧值"的现象一致（待 probe_timing.py 实测确认）。

三种模式（config.SENSE_MODE）：
  "reflection" : 只读 reflection()。等价于已验证基线；**无色标检测**。
  "rgb"        : 只读 rgb()。白边 = 分类为 white，或未识别且亮度 > RGB_EDGE_BRIGHT；
                 色标 = BoxClassifier + Debounce。需先跑 calibrate_marker.py。
  "dual"       : reflection() 判白边 + rgb() 判色标（每 tick 切两次模式，
                 只有 probe_timing.py 证明切换代价可接受才用）。
"""

import config
from fsm import BoxClassifier, Debounce, features, signature_from_dict


class Sample:
    def __init__(self):
        self.refl = None
        self.rgb = None
        self.bright = None
        self.label = None      # 单样本分类：'floor' / 'white' / 色标名 / None
        self.white = False     # 交给巡线状态机的"见白"
        self.marker = None     # 已确认（防抖后）的色标名


def build_classifier():
    sigs = []
    for name in config.SIGNATURES:
        sigs.append(signature_from_dict(name, config.SIGNATURES[name]))
    return BoxClassifier(sigs) if sigs else None


class Sensing:
    def __init__(self, color, mode=None):
        self.color = color
        self.mode = mode if mode is not None else config.SENSE_MODE
        self.refl_thr = config.edge_threshold()
        self.rgb_thr = config.RGB_EDGE_BRIGHT
        self.clf = build_classifier()
        self.markers = tuple(n for n in config.SIGNATURES if n not in ("floor", "white"))
        self.debounce = Debounce(config.MARKER_CONFIRM)
        self.marker_enabled = self.mode in ("rgb", "dual") and len(self.markers) > 0
        if self.mode in ("rgb", "dual") and self.clf is None and self.rgb_thr is None:
            raise ValueError("SENSE_MODE needs calibrate_marker.py results in config")

    def describe(self):
        return "mode={} refl_thr={} rgb_thr={} markers={}".format(
            self.mode, self.refl_thr, self.rgb_thr, self.markers)

    async def read(self):
        s = Sample()
        if self.mode == "reflection":
            s.refl = await self.color.reflection()
            s.white = s.refl > self.refl_thr
            s.label = "white" if s.white else "floor"
            return s

        if self.mode == "dual":
            s.refl = await self.color.reflection()
        s.rgb = await self.color.rgb()
        f = features(s.rgb)
        s.bright = f[3]

        if self.clf is not None:
            s.label, _ = self.clf.classify(s.rgb)

        is_marker = s.label in self.markers
        if self.mode == "dual":
            s.white = (s.refl > self.refl_thr) and not is_marker
        elif s.label == "white":
            s.white = True
        elif s.label is None and self.rgb_thr is not None:
            # 未识别的样本：亮就当白边（安全侧）
            s.white = s.bright > self.rgb_thr
        else:
            s.white = False

        if self.marker_enabled:
            s.marker = self.debounce.push(s.label if is_marker else None)
        return s

    def reset_marker(self):
        self.debounce.reset()
