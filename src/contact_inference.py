"""Dependency-free GRU inference shared by EV3 and PC parity tests."""

try:
    from umath import exp, tanh
except ImportError:
    from math import exp, tanh


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

