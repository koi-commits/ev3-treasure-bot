"""
test_motors.py — 阶段一：电机方向 + 低速基础运动
Pybricks v4.1.x, EV3. 所有带 [await] 标记的 API 都在 async 函数里 await。

安全 SAFETY：
  * 先把小车架空（轮子离地）跑 Phase 1；方向确认后再放到地面。
  * 地面测试前，前方至少留 40 cm 空地。
  * CENTER 键随时停止程序；RIGHT 键进入下一阶段。

Phases：
  1. 单轮方向：左轮、右轮各正转 1 圈 -> 两个都应让车"向前滚"
  2. DriveBase：前进 200 mm -> 后退 200 mm -> 右转 90° -> 左转 90°
  3. 连续驾驶 drive()：低速前进 2 s，停下
  4. (可选) 标定：直行 1000 mm、原地转 360°，用于修正轮径/轮距
"""

from pybricks.ev3devices import Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Stop
from pybricks.robotics import DriveBase
from pybricks.tools import run_task, wait

import config as cfg
from common import battery_check, log, wait_for_button, warn_unconfirmed

RUN_CALIBRATION = False   # 方向确认、Phase 2 通过后再改成 True

ev3 = EV3Brick()
left = Motor(cfg.LEFT_MOTOR_PORT, cfg.LEFT_MOTOR_DIRECTION, gears=cfg.GEARS)
right = Motor(cfg.RIGHT_MOTOR_PORT, cfg.RIGHT_MOTOR_DIRECTION, gears=cfg.GEARS)


async def phase1_single_wheels():
    log(ev3, "P1 single wheel")
    log(ev3, "Lift robot!")
    await wait_for_button(ev3)

    for name, m in (("LEFT", left), ("RIGHT", right)):
        m.reset_angle(0)
        log(ev3, name, "+360 deg")
        await m.run_angle(cfg.TEST_WHEEL_SPEED, 360, then=Stop.HOLD)
        log(ev3, name, "angle:", m.angle())
        await wait(500)
        m.stop()

    log(ev3, "Both forward?")
    log(ev3, "No -> fix config")


async def phase2_drivebase(robot):
    log(ev3, "P2 DriveBase")
    log(ev3, "Put on floor")
    await wait_for_button(ev3)

    robot.reset()
    d = cfg.TEST_DISTANCE_MM
    a = cfg.TEST_TURN_DEG

    log(ev3, "Forward", d)
    await robot.straight(d)
    log(ev3, "dist:", robot.distance())
    await wait(500)

    log(ev3, "Backward", d)
    await robot.straight(-d)
    log(ev3, "dist:", robot.distance())
    await wait(500)

    # DriveBase: 正角度 = 右转 (顺时针，俯视)
    log(ev3, "Turn right", a)
    await robot.turn(a)
    log(ev3, "angle:", robot.angle())
    await wait(500)

    log(ev3, "Turn left", a)
    await robot.turn(-a)
    log(ev3, "angle:", robot.angle())
    robot.stop()


async def phase3_continuous(robot):
    log(ev3, "P3 drive()")
    await wait_for_button(ev3)

    robot.reset()
    robot.drive(cfg.TEST_STRAIGHT_SPEED, 0)    # drive() 不是 awaitable
    await wait(cfg.TEST_DRIVE_TIME_MS)
    robot.stop()
    log(ev3, "dist:", robot.distance())
    log(ev3, "angle:", robot.angle())


async def phase4_calibration(robot):
    log(ev3, "P4 calibration")
    log(ev3, "Mark start, 1m")
    await wait_for_button(ev3)
    robot.reset()
    await robot.straight(1000, then=Stop.BRAKE)
    log(ev3, "Measure real mm")

    log(ev3, "Turn 360 next")
    await wait_for_button(ev3)
    robot.reset()
    await robot.turn(360, then=Stop.BRAKE)
    log(ev3, "Check heading")


async def main():
    ev3.screen.clear()
    battery_check(ev3, cfg.LOW_BATTERY_MV)
    warn_unconfirmed(ev3, cfg)

    await phase1_single_wheels()

    robot = DriveBase(left, right, cfg.WHEEL_DIAMETER_MM, cfg.AXLE_TRACK_MM)
    robot.settings(
        straight_speed=cfg.TEST_STRAIGHT_SPEED,
        straight_acceleration=cfg.TEST_STRAIGHT_ACCEL,
        turn_rate=cfg.TEST_TURN_RATE,
        turn_acceleration=cfg.TEST_TURN_ACCEL,
    )
    log(ev3, "settings:", robot.settings())

    await phase2_drivebase(robot)
    await phase3_continuous(robot)
    if RUN_CALIBRATION:
        await phase4_calibration(robot)

    robot.stop()
    log(ev3, "Motor test done")
    await wait(3000)


run_task(main())
