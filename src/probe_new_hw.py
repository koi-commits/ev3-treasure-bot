r"""
probe_new_hw.py — 2026-11-17 硬件 v3 新传感器验收（陀螺仪 / 触感 / 颜色）

用 SECTIONS 常量选段（默认三段全跑）：

  a) 陀螺仪静止漂移：等待 2 s -> gyro.reset_angle(0) -> 打印 3 s 与 30 s 的读数。
     通过标准：漂移 < 2 deg/min（30 s 内 < 1 deg）。车要放在水平的黑场中间，别碰。
  b) 20 s 活体打印（10 Hz）：触感 / 陀螺 / 反射率。开头只打印一次 touch.pressed() 与
      gyro.angle() 的原始返回类型，用来实机确认这两个读值到底要不要 await。
     所有读值都经 common.aread()（数字原样返回，awaitable 才 await）。
     用法：手指按住推板顶点的触感；手动转车身看陀螺仪；把光斑依次放到黑场 / 白线上。
  c) 原地转 360°（RIGHT 确认后才动）：比较 gyro.angle() 与 drive.angle()。
     打印符号是否一致、比例 k、建议的 AXLE_TRACK_MM = 104 * k。
     通过标准：|k - 1| < 0.05 时不用改；否则把建议值写回 config.AXLE_TRACK_MM 再测。

约束（与 mission.py 一致）：
  * 每个 awaitable 都 await；整段只读一种颜色模式（reflection），不跨模式切换。
  * 设备只在 hardware.py 顶层构造；这里只 import。
  * RIGHT = 下一步 / 放行动作，CENTER = Pybricks 急停。
"""

import config

from common import aread, battery_check, log, wait_for_button, warn_unconfirmed
from hardware import color, drive, ev3, gyro, touch
from pybricks.tools import run_task, wait

# ---- 选择要跑的段：'a' / 'ab' / 'c' ... ----
SECTIONS = "abc"

DRIFT_SETTLE_MS = 2000          # 上电后先静置，陀螺仪输出稳定
DRIFT_SHORT_MS = 3000
DRIFT_LONG_MS = 30000
DRIFT_PASS_DEG = 1.0            # 30 s 内漂移 < 1 deg  <=>  2 deg/min
LIVE_MS = 20000
LIVE_PERIOD_MS = 100            # 10 Hz
TURN_DEG = 360


def _fmt(value, digits=1):
    """None 也要能打印（传感器缺失时）。"""
    if value is None:
        return "-"
    if digits >= 2:
        return "{:.2f}".format(value)
    return "{:.1f}".format(value)


async def sec_drift():
    log(ev3, "--- a) gyro drift ---")
    if gyro is None:
        log(ev3, "FAIL: gyro is None (check GYRO_SENSOR_PORT / cable)")
        return
    log(ev3, "settle", DRIFT_SETTLE_MS, "ms ...")
    await wait(DRIFT_SETTLE_MS)
    gyro.reset_angle(0)
    await wait(DRIFT_SHORT_MS)
    short = await aread(gyro.angle())
    log(ev3, "drift 3s  =", _fmt(short, 2), "deg")
    await wait(DRIFT_LONG_MS - DRIFT_SHORT_MS)
    long = await aread(gyro.angle())
    log(ev3, "drift 30s =", _fmt(long, 2), "deg =", _fmt(long * 2.0, 2), "deg/min")
    if abs(long) < DRIFT_PASS_DEG:
        log(ev3, "PASS: drift < 2 deg/min")
    else:
        log(ev3, "FAIL: drift >= 2 deg/min -> 车放平/静置再测；仍失败则改用陀螺仪前先 reset")


async def sec_live():
    log(ev3, "--- b) live", LIVE_MS // 1000, "s @", 1000 // LIVE_PERIOD_MS, "Hz ---")
    log(ev3, "press touch / rotate body / move spot over floor and white line")
    # 只打印一次原始返回类型：实机跑这一行就能定死"触感/陀螺读值到底要不要 await"。
    if touch is not None and gyro is not None:
        t_type = type(touch.pressed())   # aread-ok: 只取类型，不读值、不影响后续
        g_type = type(gyro.angle())      # aread-ok: 只取类型，不读值、不影响后续
        log(ev3, "type touch:", t_type, "gyro:", g_type)
    else:
        log(ev3, "WARN: sensor missing, type probe skipped")
    t = 0
    while t < LIVE_MS:
        pressed = None
        angle = None
        if touch is not None:
            pressed = await aread(touch.pressed())
        if gyro is not None:
            angle = await aread(gyro.angle())
        refl = await color.reflection()
        log(ev3, "t=", t, "touch=", pressed, "gyro=", _fmt(angle, 1), "refl=", _fmt(refl, 1))
        await wait(LIVE_PERIOD_MS)
        t += LIVE_PERIOD_MS
    log(ev3, "HINT: touch 应为 False/True 跟随手指；白线上 refl 应 >",
        _fmt(config.edge_threshold(), 2), "(当前阈值)")


async def sec_turn():
    log(ev3, "--- c) turn", TURN_DEG, "deg: gyro vs drive ---")
    if gyro is None:
        log(ev3, "FAIL: gyro is None -> 无法比较（drive.angle() 只是轮子推算）")
        return
    await wait_for_button(ev3, prompt="RIGHT = turn 360 (need ~0.5 m clear space)")
    drive.reset()
    await wait(300)                     # reset 后让车身完全停稳
    gyro.reset_angle(0)
    await wait(300)
    g0 = await aread(gyro.angle())
    d0 = drive.angle()
    await drive.turn(TURN_DEG)
    await wait(500)                     # 停下后车身还会轻微回弹，等一下再读
    gd = (await aread(gyro.angle())) - g0
    dd = drive.angle() - d0
    log(ev3, "gyro delta =", _fmt(gd, 1), "deg   drive delta =", _fmt(dd, 1), "deg")

    if abs(gd) < 30:
        log(ev3, "FAIL: gyro almost did not move -> 检查 GYRO_SENSOR_PORT / 车是否真的转了")
        return

    if (gd > 0) == (dd > 0):
        log(ev3, "sign: SAME -> GYRO_POSITIVE_DIRECTION = 1")
    else:
        log(ev3, "sign: OPPOSITE -> GYRO_POSITIVE_DIRECTION = -1")

    # drive.angle() = 轮子行程 / 配置轮距；真实转角 = 轮子行程 / 真实轮距
    # => 真实轮距 = 配置轮距 * (drive 读数 / 真实读数)
    k = dd / gd
    suggested = config.AXLE_TRACK_MM * k
    log(ev3, "k = drive/gyro =", _fmt(k, 4))
    log(ev3, "suggested AXLE_TRACK_MM =", _fmt(config.AXLE_TRACK_MM, 2), "*", _fmt(k, 4),
        "=", _fmt(suggested, 1))
    if abs(k - 1.0) < 0.05:
        log(ev3, "PASS: |k-1| < 0.05, 轮距可用")
    else:
        log(ev3, "ADJUST: 写回 config.AXLE_TRACK_MM =", _fmt(suggested, 1), "再跑一次 c 段")


async def main():
    ev3.screen.clear()
    log(ev3, "=== probe_new_hw ===")
    log(ev3, "sections:", SECTIONS)
    battery_check(ev3, config.LOW_BATTERY_MV)
    warn_unconfirmed(ev3, config)
    if touch is None:
        log(ev3, "WARN: touch is None")
    if gyro is None:
        log(ev3, "WARN: gyro is None")

    if "a" in SECTIONS:
        await sec_drift()
    if "b" in SECTIONS:
        await sec_live()
    if "c" in SECTIONS:
        await sec_turn()

    log(ev3, "=== done ===")


run_task(main())
