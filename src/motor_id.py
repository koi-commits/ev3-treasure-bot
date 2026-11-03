r"""
motor_id.py — 识别 A / D 哪个是左轮、哪个是右轮，以及"正速度"是不是前进。

    pbrun.cmd src\motor_id.py

安全要求：把小车**架空**（垫起来轮子离地）再跑。

流程（慢速 150 deg/s，每个电机转 2 秒，中间停 2.5 秒）：
  1. 单响长声 + 屏幕显示 "A >>"  -> 只转 A 口电机
  2. 停 2.5 秒（屏幕 "wait"）
  3. 两声短响 + 屏幕显示 "D >>"  -> 只转 D 口电机
  4. 停

观察要点：
  * 哪一次转的是左轮、哪一次是右轮？
  * 轮子是往车头方向滚（前进）还是往车尾方向滚（后退）？
"""

import config
from hardware import ev3, left, right

from pybricks.tools import run_task, wait

SPEED = 150       # deg/s，比上次更慢，方便看清
RUN_MS = 2000


async def main():
    ev3.screen.clear()
    print("motor_id: A then D, 150 deg/s, 2.0 s each")

    a0 = left.angle()
    d0 = right.angle()

    # ---- A ----
    ev3.speaker.beep(900, 500)
    await wait(800)
    ev3.screen.clear()
    ev3.screen.print("A >>")
    print("--- Port.A (config LEFT) spinning ---")
    left.run(SPEED)
    await wait(RUN_MS)
    left.brake()
    print("A angle delta:", left.angle() - a0)

    # ---- 停顿，留出观察时间 ----
    ev3.screen.clear()
    ev3.screen.print("wait")
    print("--- pause 2.5 s ---")
    await wait(2500)

    # ---- D ----
    ev3.speaker.beep(900, 200)
    await wait(150)
    ev3.speaker.beep(900, 200)
    await wait(800)
    ev3.screen.clear()
    ev3.screen.print("D >>")
    print("--- Port.D (config RIGHT) spinning ---")
    right.run(SPEED)
    await wait(RUN_MS)
    right.brake()
    print("D angle delta:", right.angle() - d0)

    await wait(500)
    ev3.speaker.beep(500, 700)
    ev3.screen.clear()
    ev3.screen.print("done")
    print("done")


run_task(main())
