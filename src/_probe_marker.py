r"""
_probe_marker.py — 只测终点色标（临时诊断脚本，不属于交付代码）。

背景（2026-11-10 实机）：calibrate_marker.py 采到的 "finish" 是
    rgb_mean=(37.5, 6.7, 6.5)
而 LIVE 里真正压在红色色标上时读到的是
    rgb=(58, 20, 21)
→ g/b 远超标定容差，BoxClassifier 全判 label=None，且 bright≈33 > RGB_EDGE_BRIGHT=25，
  会让色标被当成白边。说明标定时那块红色的位置/光照与实际不符。

本脚本只做一件事：把光斑准确压在色标上，采 RGB_N 个样本，
直接用 build_signature 生成 can-paste 的签名行。

    pbrun.cmd src\_probe_marker.py
"""

import config
from common import log
from fsm import BoxClassifier, build_signature, features, signature_from_dict
from hardware import color, ev3

from pybricks.parameters import Button, Color
from pybricks.tools import run_task, wait

RGB_N = 50


async def wait_right(prompt):
    log(ev3, prompt)
    print(prompt)
    while Button.RIGHT not in ev3.buttons.pressed():
        await wait(30)
    while Button.RIGHT in ev3.buttons.pressed():
        await wait(30)
    await wait(300)


async def sample(n):
    out = []
    for _ in range(n):
        out.append(await color.rgb())
        await wait(config.SAMPLE_PERIOD_MS)
    return out


async def main():
    print("=== probe marker ===")
    print("current SIGNATURES:")
    for name in config.SIGNATURES:
        print("  ", name, config.SIGNATURES[name])
    print("RGB_EDGE_BRIGHT =", config.RGB_EDGE_BRIGHT)

    # 先用现有签名看一遍实时分类，确认"现在判不出来"这个现象
    clf = BoxClassifier([signature_from_dict(n, config.SIGNATURES[n]) for n in config.SIGNATURES])
    await wait_right("put spot on RED, then RIGHT")

    # 采样期间实时打印，用户能看到贴没贴准
    samples = []
    for i in range(RGB_N):
        rgb = await color.rgb()
        samples.append(rgb)
        if i % 5 == 0:
            lab, d = clf.classify(rgb)
            print("  rgb={} bright={:.1f} -> {} d={:.2f}".format(
                rgb, features(rgb)[3], lab, d))
        await wait(config.SAMPLE_PERIOD_MS)

    br = [features(s)[3] for s in samples]
    sig = build_signature("finish", samples)
    print("--- raw ---")
    print("n=", len(samples), "bright", min(br), "..", max(br))
    print("samples:", samples)
    print("--- signature ---")
    print('    "finish": {{"mean": {}, "tol": {}}},'.format(
        tuple(round(v, 1) for v in sig.mean), tuple(round(v, 1) for v in sig.tol)))
    print("--- re-classify all samples with the NEW signature ---")
    sigs = [sig]
    for n in config.SIGNATURES:
        if n != "finish":
            sigs.append(signature_from_dict(n, config.SIGNATURES[n]))
    clf2 = BoxClassifier(sigs)
    hit = 0
    for s in samples:
        lab, d = clf2.classify(s)
        if lab == "finish":
            hit += 1
    print("finish recognized {}/{}".format(hit, len(samples)))
    ev3.speaker.beep(900, 200)
    print("done")


run_task(main())
