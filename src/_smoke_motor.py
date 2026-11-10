r"""
_smoke_motor.py — 最小电机连通性检查（临时诊断脚本，不属于交付代码）。

目的：demo_edge.py 在按 RIGHT 之后立刻出现
      "The hub disconnected (the hub stopped answering)"，
而 test_color.py（不碰电机）可以稳定跑完 10 s。
本脚本逐级加负载，定位是"电机一动就掉线"还是别的：

  S0  只打印、不动电机（对照）
  S1  按 RIGHT 后 drive.straight(80 mm)，低速
  S2  drive.drive(150 mm/s, 0) 跑 1 s
  S3  drive.turn(90)

每步前后都打印，掉线时能从日志看出死在哪一步。
"""

import config
from hardware import drive, ev3, left, right

from pybricks.parameters import Button, Color
from pybricks.tools import run_task, wait


async def wait_right(msg):
    ev3.screen.clear()
    ev3.screen.print(msg)
    print(msg)
    while Button.RIGHT not in ev3.buttons.pressed():
        await wait(30)
    while Button.RIGHT in ev3.buttons.pressed():
        await wait(30)
    print("go")
    await wait(300)


async def main():
    print("=== smoke motor ===")
    print("battery", ev3.battery.voltage())
    print("S0 alive (no motor motion)")
    await wait(1000)
    print("S0 ok")

    await wait_right("S1 straight 80mm")
    print("S1 start")
    await drive.straight(80)
    print("S1 done, dist=", int(drive.distance()))

    await wait_right("S2 drive 1s")
    print("S2 start")
    drive.drive(150, 0)
    await wait(1000)
    drive.brake()
    print("S2 done, dist=", int(drive.distance()))

    ev3.speaker.beep(700, 200)
    print("smoke done")
    ev3.screen.clear()
    ev3.screen.print("done")


run_task(main())
