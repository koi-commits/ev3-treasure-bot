r"""
calib.py — 色标标定的纯分析逻辑（无硬件依赖，离线可测）。

输入：每个表面的一组 rgb 样本（以及可选的 reflection 样本）
输出：SIGNATURES、RGB_EDGE_BRIGHT、PASS/FAIL 判定与原因、可直接粘贴进 config.py 的文本
"""

from fsm import BoxClassifier, build_signature, features, overlaps

EDGE_NAMES = ("floor", "white")


def _mix(a, b, k):
    return tuple(a[i] * (1 - k) + b[i] * k for i in range(3))


def _mean_rgb(samples):
    n = len(samples)
    return tuple(sum(s[i] for s in samples) / n for i in range(3))


def analyze(rgb_by_name, refl_by_name=None, refl_threshold=None):
    """返回 dict：signatures, rgb_edge_bright, ok, problems(list), notes(list), lines(list)"""
    problems = []
    notes = []
    for need in EDGE_NAMES:
        if need not in rgb_by_name:
            problems.append("missing surface: " + need)
    if problems:
        return {"ok": False, "problems": problems, "notes": notes,
                "signatures": {}, "rgb_edge_bright": None, "lines": problems[:]}

    sigs = {}
    for name in rgb_by_name:
        sigs[name] = build_signature(name, rgb_by_name[name])
    markers = [n for n in sigs if n not in EDGE_NAMES]

    # 1. 两两重叠
    names = list(sigs)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if overlaps(sigs[names[i]], sigs[names[j]]):
                problems.append("overlap: {} ~ {}".format(names[i], names[j]))

    # 2. 黑白过渡的混合光斑不能被识别成色标
    clf = BoxClassifier([sigs[n] for n in names])
    fm = _mean_rgb(rgb_by_name["floor"])
    wm = _mean_rgb(rgb_by_name["white"])
    for step in range(21):
        k = step / 20.0
        lab, _ = clf.classify(_mix(fm, wm, k))
        if lab in markers:
            problems.append("floor/white mix {:.0f}% looks like {}".format(k * 100, lab))
            break

    # 3. 亮度阈值：地面最亮 与 白边最暗 的中点
    fb = [features(s)[3] for s in rgb_by_name["floor"]]
    wb = [features(s)[3] for s in rgb_by_name["white"]]
    gap = min(wb) - max(fb)
    edge_bright = (max(fb) + min(wb)) / 2.0
    if gap < 3:
        problems.append("floor/white brightness gap only {:.1f}".format(gap))

    # 4. 色标的亮度是否会被当成白边（在 rgb 模式下由分类器优先处理，仅提示）
    for m in markers:
        mb = sigs[m].mean[3]
        if mb > edge_bright:
            notes.append("{} brighter than edge threshold: relies on classifier (ok in rgb mode)".format(m))

    # 5. reflection 模式下色标是否会触发 ESCAPE
    if refl_by_name and refl_threshold is not None:
        for m in markers:
            vals = refl_by_name.get(m)
            if vals:
                hi = max(vals)
                if hi > refl_threshold:
                    notes.append("{} reflection max {:.1f} > edge thr {:.2f}: reflection mode would treat it as edge".format(
                        m, hi, refl_threshold))

    if not markers:
        notes.append("no marker surface sampled: edge detection only")

    sig_out = {}
    for n in names:
        s = sigs[n]
        sig_out[n] = {"mean": tuple(round(v, 1) for v in s.mean),
                      "tol": tuple(round(v, 1) for v in s.tol)}

    lines = ['SENSE_MODE = "rgb"',
             "RGB_EDGE_BRIGHT = {:.1f}".format(edge_bright),
             "SIGNATURES = {"]
    for n in names:
        lines.append('    "{}": {{"mean": {}, "tol": {}}},'.format(n, sig_out[n]["mean"], sig_out[n]["tol"]))
    lines.append("}")
    return {"ok": not problems, "problems": problems, "notes": notes,
            "signatures": sig_out, "rgb_edge_bright": edge_bright, "lines": lines}
