r"""
calibrate_geometry.py — 轮径 / 轮距实测（官方方法）。会驱动电机，需要 ≥ 1.2 m 直线空地。

    pbrun.cmd src\calibrate_geometry.py

  1. STRAIGHT：在地上贴一条起始线，对齐**两轮轴心**。RIGHT -> 慢速直行 1000 mm。
     用卷尺量实际距离 A（mm）。
        新 WHEEL_DIAMETER_MM = 当前值 × A / 1000
  2. TURN：在车头方向贴一条参考线。RIGHT -> 原地转 720°（两圈，放大误差）。
     看停下时车头与参考线的夹角：
        转多了 e 度（实际 720+e）：新 AXLE_TRACK_MM = 当前值 × 720 / (720 + e)
        转少了 e 度（实际 720-e）：新 AXLE_TRACK_MM = 当前值 × 720 / (720 - e)
     先改轮径再测轮距；改完两项各复测一次，误差 < 1% 即把 GEOMETRY_CONFIRMED 设为 True。
  3. SQUARE（可选）：300 mm 边长正方形走一圈，看回到起点的偏差。

为什么重要：full 模式的扫描线平行度（绝对航向）和里程计送货都靠这两个数。
沿边巡线不依赖它们。
"""

import config
from common import battery_check, log, wait_button_or_timeout
from hardware import drive, ev3

from pybricks.parameters import Stop
from pybricks.tools import run_task, wait


async def main():
    ev3.screen.clear()
    log(ev3, "GEOMETRY CAL")
    battery_check(ev3, config.LOW_BATTERY_MV)
    print("current WHEEL_DIAMETER_MM =", config.WHEEL_DIAMETER_MM,
          " AXLE_TRACK_MM =", config.AXLE_TRACK_MM)
    drive.settings(straight_speed=120, straight_acceleration=200,
                   turn_rate=90, turn_acceleration=200)

    if await wait_button_or_timeout(ev3, 60000, prompt="STRAIGHT 1m: RIGHT"):
        drive.reset()
        await drive.straight(1000, then=Stop.HOLD)
        log(ev3, "dist", drive.distance())
        print("Measure actual A mm. new WHEEL_DIAMETER_MM = {} * A / 1000".format(
            config.WHEEL_DIAMETER_MM))
        drive.stop()

    if await wait_button_or_timeout(ev3, 60000, prompt="TURN 720: RIGHT"):
        drive.reset()
        await drive.turn(720, then=Stop.HOLD)
        log(ev3, "angle", drive.angle())
        print("Measure error e. over: new AXLE = {0} * 720/(720+e); short: {0} * 720/(720-e)".format(
            config.AXLE_TRACK_MM))
        drive.stop()

    if await wait_button_or_timeout(ev3, 20000, prompt="SQUARE (opt): RIGHT"):
        for _ in range(4):
            await drive.straight(300)
            await drive.turn(90)
        drive.stop()
        log(ev3, "square done")

    ev3.speaker.beep(500, 300)
    await wait(500)


run_task(main())
