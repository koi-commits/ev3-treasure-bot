"""Dependency-free GRU inference shared by EV3 and PC parity tests."""

# 2026-11-10 实机发现（部署阻断）：EV3 上这段 import 直接抛
#     ImportError: can't import name tanh
# 因为 Pybricks 的 MicroPython 构建里 `math` 没有 tanh（标准 MicroPython 也没有）。
# 原代码只写 `from math import exp, tanh`，在 CPython 上永远是通的，
# 所以 PC 测试全绿、实机一跑就崩 —— 这正是"PC 跑通不等于部署成功"的实例。
# 修法：只在缺 tanh 时补一个纯 Python 实现（适配层，不改模型与数值语义）。
try:
    from umath import exp, tanh
except ImportError:
    from math import exp

    def tanh(x):
        """tanh 的纯 Python 近似实现（EV3 的 math 没有 tanh）。

        用 2*sigmoid(2x)-1 的等价形式，并在 |x| 较大时饱和，
        避免 exp 溢出。与 CPython math.tanh 的最大误差 < 1e-7。
        """
        if x >= 15.0:
            return 1.0
        if x <= -15.0:
            return -1.0
        e = exp(-2.0 * x)
        return (1.0 - e) / (1.0 + e)


def _sigmoid(x):
    if x >= 0:
        return 1.0 / (1.0 + exp(-x))
    e = exp(x)
    return e / (1.0 + e)


def _affine(rows, x, bias):
    out = []
    for i in range(len(rows)):
        row = rows[i]
        s = bias[i]
        for j in range(len(x)):
            s += row[j] * x[j]
        out.append(s)
    return out


class ContactInference:
    """PyTorch-compatible GRUCell + sigmoid capture head."""

    def __init__(self, weights):
        self.w = weights
        self.h = [0.0] * weights.HIDDEN_SIZE
        self.run = 0

    def reset(self):
        for i in range(len(self.h)):
            self.h[i] = 0.0
        self.run = 0

    def update(self, features):
        w = self.w
        gi = _affine(w.W_IH, features, w.B_IH)
        gh = _affine(w.W_HH, self.h, w.B_HH)
        n = w.HIDDEN_SIZE
        new_h = [0.0] * n
        for i in range(n):
            r = _sigmoid(gi[i] + gh[i])
            z = _sigmoid(gi[n + i] + gh[n + i])
            candidate = tanh(gi[2 * n + i] + r * gh[2 * n + i])
            new_h[i] = (1.0 - z) * candidate + z * self.h[i]
        self.h = new_h
        logit = w.B_OUT
        for i in range(n):
            logit += w.W_OUT[i] * new_h[i]
        probability = _sigmoid(logit)
        self.run = self.run + 1 if probability >= w.THRESHOLD else 0
        return probability, self.run >= w.DEBOUNCE_N

