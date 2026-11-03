r"""
calibrate_marker.py — 终点色标标定（RGB 单模式）。不驱动电机。

    pbrun.cmd src\calibrate_marker.py

每个表面：把车放上去，按 RIGHT；采样期间**用手把车在该表面上慢慢前后/左右挪动 1–2 cm**，
让样本覆盖真实行驶时的轻微变化（光斑位置、车身晃动）。
  1. FLOOR   黑色场地
  2. WHITE   白色边界
  3. FINISH  终点色标（必须是最终比赛用的那块色标，放在比赛场地上）
  4. EXTRA   可选：其他会出现在地面的颜色（例如第二种色标/胶带）。不需要就等超时跳过。

每个表面先采 RGB_N 个 rgb()，再单独采 REFL_N 个 reflection()（两段之间只切一次模式）。
最后打印：每个表面统计、PASS/FAIL、可直接粘贴进 config.py 的 SENSE_MODE/RGB_EDGE_BRIGHT/SIGNATURES。
然后进入 LIVE 模式：只读 rgb()，屏幕实时显示分类结果，手推小车跨过各表面检查。CENTER 结束。
"""

import config
from calib import analyze
from common import log, wait_button_or_timeout
from fsm import BoxClassifier, Debounce, features, signature_from_dict
from hardware import color, ev3

from pybricks.parameters import Color
from pybricks.tools import run_task, wait

RGB_N = 40
REFL_N = 20
SURFACES = (("floor", 60000), ("white", 60000), ("finish", 60000), ("extra", 15000))


async def sample(name):
    rgbs = []
    for _ in range(RGB_N):
        rgbs.append(await color.rgb())
        await wait(config.SAMPLE_PERIOD_MS)
    refls = []
    for _ in range(REFL_N):
        refls.append(await color.reflection())
        await wait(config.SAMPLE_PERIOD_MS)
    br = [features(s)[3] for s in rgbs]
    m = tuple(sum(s[i] for s in rgbs) / len(rgbs) for i in range(3))
    print("{:7s} rgb_mean=({:.1f},{:.1f},{:.1f}) bright {:.1f}..{:.1f}  refl {:.1f}..{:.1f}".format(
        name, m[0], m[1], m[2], min(br), max(br), min(refls), max(refls)))
    print("   raw rgb:", rgbs)
    return rgbs, refls


async def live(clf):
    log(ev3, "LIVE (CENTER=quit)")
    deb = Debounce(config.MARKER_CONFIRM)
    n = 0
    while True:
        rgb = await color.rgb()
        lab, d = clf.classify(rgb)
        conf = deb.push(lab)
        n += 1
        if n % 4 == 0:
            ev3.screen.clear()
            ev3.screen.print(str(lab))
            ev3.screen.print("ok" if conf else "..")
            ev3.screen.print(str(rgb))
            print("rgb={} label={} d={:.2f} confirmed={}".format(rgb, lab, d, conf))
        ev3.light.on(Color.GREEN if lab == "floor" else Color.ORANGE if lab is None else Color.RED)
        await wait(config.SAMPLE_PERIOD_MS)


async def main():
    ev3.screen.clear()
    log(ev3, "MARKER CAL")
    rgb_by = {}
    refl_by = {}
    for name, timeout in SURFACES:
        ev3.speaker.beep(800, 100)
        pressed = await wait_button_or_timeout(ev3, timeout, prompt="on " + name.upper() + ", RIGHT")
        if not pressed:
            log(ev3, "skip", name)
            continue
        log(ev3, "sampling", name)
        rgb_by[name], refl_by[name] = await sample(name)
        ev3.speaker.beep(1200, 100)

    rep = analyze(rgb_by, refl_by, config.edge_threshold())
    print("==== RESULT:", "PASS" if rep["ok"] else "FAIL", "====")
    for p in rep["problems"]:
        print("PROBLEM:", p)
    for n in rep["notes"]:
        print("note:", n)
    print("---- paste into src/config.py (section 9) ----")
    for line in rep["lines"]:
        print(line)
    print("----")
    ev3.screen.clear()
    log(ev3, "PASS" if rep["ok"] else "FAIL")

    if rep["signatures"]:
        sigs = [signature_from_dict(n, rep["signatures"][n]) for n in rep["signatures"]]
        await wait(1000)
        await live(BoxClassifier(sigs))


run_task(main())
