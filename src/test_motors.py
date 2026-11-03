r"""
test_motors.py — 阶段一：电机与几何验证（分 3 步，逐步放开）。

    pbrun.cmd src\test_motors.py

每步必须按 RIGHT 才执行；不会超时自动驱动电机。
安全：把小车**架空**（轮子离地）或放在地板中央开阔处再开始。

步骤：
  1. 单轮识别：先转"左"电机再转"右"电机，各 1 秒 / 200 deg/s
  2. 直行 200 mm，再后退 200 mm
  3. 原地左转 90°，再右转 90°
"""

import config
from common import battery_check, log, wait_for_button
from hardware import drive, ev3, left, right

from pybricks.tools import run_task, wait


async def phase1():
    log(ev3, "--- 1 single wheel ---")
    await wait_for_button(ev3, prompt="RIGHT: spin L then R")

    a0 = left.angle()
    left.run(config.TEST_WHEEL_SPEED)
    await wait(1000)
    left.brake()
    await wait(600)
    log(ev3, "L motor moved", left.angle() - a0, "deg")

    d0 = right.angle()
    right.run(config.TEST_WHEEL_SPEED)
    await wait(1000)
    right.brake()
    await wait(400)
    log(ev3, "R motor moved", right.angle() - d0, "deg")


async def phase2():
    log(ev3, "--- 2 straight ---")
    await wait_for_button(ev3, prompt="RIGHT: straight 200mm")

    drive.reset()
    drive.settings(config.TEST_STRAIGHT_SPEED, config.TEST_STRAIGHT_ACCEL)
    await drive.straight(config.TEST_DISTANCE_MM)
    await wait(500)
    log(ev3, "fwd -> distance", int(drive.distance()), "mm (target",
        config.TEST_DISTANCE_MM, ")")

    await drive.straight(-config.TEST_DISTANCE_MM)
    await wait(500)
    log(ev3, "back -> distance", int(drive.distance()), "mm (target 0)")


async def phase3():
    log(ev3, "--- 3 turn ---")
    await wait_for_button(ev3, prompt="RIGHT: turn 90 deg")

    drive.reset()
    drive.settings(turn_rate=config.TEST_TURN_RATE, turn_acceleration=config.TEST_TURN_ACCEL)
    await drive.turn(config.TEST_TURN_DEG)
    await wait(500)
    log(ev3, "turn L -> angle", int(drive.angle()), "deg (target",
        config.TEST_TURN_DEG, ")")

    await drive.turn(-config.TEST_TURN_DEG)
    await wait(500)
    log(ev3, "turn R -> angle", int(drive.angle()), "deg (target 0)")


async def main():
    ev3.screen.clear()
    log(ev3, "=== test_motors ===")
    battery_check(ev3, config.LOW_BATTERY_MV)
    log(ev3, "L:", config.LEFT_MOTOR_PORT, "R:", config.RIGHT_MOTOR_PORT)

    await phase1()
    await phase2()
    await phase3()

    drive.settings(config.TEST_STRAIGHT_SPEED, config.TEST_STRAIGHT_ACCEL)
    drive.stop()
    ev3.speaker.beep(500, 400)
    log(ev3, "=== done ===")


run_task(main())
