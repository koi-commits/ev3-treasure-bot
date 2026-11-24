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

两个传感器（2026-11-24）：左 = color（巡线、旧行为全部基于它），右 = color_right（可为 None）。
每个 tick 两个都读，**两个都用同一种模式**，绝不在同一个传感器上轮流切模式。
Sample 的老字段（white / label / marker / rgb / refl）仍然只代表左传感器；
右传感器的结果在 sample.right（同样的 Sample），合并结果见 boundary_any()。
"""

import config
from fsm import BoxClassifier, Debounce, features, is_red, signature_from_dict


class Sample:
    def __init__(self):
        self.refl = None
        self.rgb = None
        self.bright = None
        self.label = None      # 单样本分类：'floor' / 'white' / 色标名 / None
        self.white = False     # 交给巡线状态机的"见白"
        self.marker = None     # 已确认（防抖后）的色标名
        self.marker_side = None  # 确认色标的那个传感器：0 左 / 1 右
        self.right = None      # 右传感器的 Sample（没有右传感器时 None）


def build_classifier():
    sigs = []
    for name in config.SIGNATURES:
        sigs.append(signature_from_dict(name, config.SIGNATURES[name]))
    return BoxClassifier(sigs) if sigs else None


class Sensing:
    def __init__(self, color, mode=None, color_right=None):
        self.color = color
        self.color_right = color_right
        self.mode = mode if mode is not None else config.SENSE_MODE
        self.refl_thr = config.edge_threshold()
        self.rgb_thr = config.RGB_EDGE_BRIGHT
        self.clf = build_classifier()
        self.markers = tuple(n for n in config.SIGNATURES if n not in ("floor", "white"))
        self.debounce = Debounce(config.MARKER_CONFIRM)
        self.debounce_r = Debounce(config.MARKER_CONFIRM)
        self.marker_enabled = self.mode in ("rgb", "dual") and len(self.markers) > 0
        self.red_rule = config.RED_RULE if self.mode in ("rgb", "dual") else None
        if self.mode in ("rgb", "dual") and self.clf is None and self.rgb_thr is None:
            raise ValueError("SENSE_MODE needs calibrate_marker.py results in config")

    def describe(self):
        return "mode={} refl_thr={} rgb_thr={} markers={} sensors={}".format(
            self.mode, self.refl_thr, self.rgb_thr, self.markers,
            2 if self.color_right is not None else 1)

    async def read(self):
        s = await self._read_one(self.color, self.debounce)
        if self.color_right is not None:
            s.right = await self._read_one(self.color_right, self.debounce_r)
            if s.marker is None and s.right.marker is not None:
                s.marker = s.right.marker
                s.marker_side = 1
            elif s.marker is not None:
                s.marker_side = 0
        elif s.marker is not None:
            s.marker_side = 0
        return s

    async def _read_one(self, color, debounce):
        s = Sample()
        if self.mode == "reflection":
            s.refl = await color.reflection()
            s.white = s.refl > self.refl_thr
            s.label = "white" if s.white else "floor"
            return s

        if self.mode == "dual":
            s.refl = await color.reflection()
        s.rgb = await color.rgb()
        f = features(s.rgb)
        s.bright = f[3]

        if self.clf is not None:
            s.label, _ = self.clf.classify(s.rgb)

        # 红色比例兜底：盒子没命中但明显偏红，也算终点色标（见 fsm.is_red 的实测依据）
        if s.label is None and self.red_rule is not None:
            if is_red(s.rgb, self.red_rule[0], self.red_rule[1], self.red_rule[2]):
                s.label = config.FINISH_LABEL

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
            s.marker = debounce.push(s.label if is_marker else None)
        return s

    def reset_marker(self):
        self.debounce.reset()
        self.debounce_r.reset()
