r"""
mission.py — 自主寻宝最小可运行版本（阶段三）。

    pbrun.cmd src\mission.py

状态机：
  EXPLORE  直行探索；看到白边 -> 后退转向；随机改变方向
  PUSH     颜色传感器读到目标物颜色 -> 直行推一段（"运输"）
  FINISH   读到终端色标 -> 停车、鸣笛、结束
  TIMEOUT  超时退出

所有阈值来自 config.py。FINISH_COLOR 未标定时，本程序只做"探索 + 避边"，
并额外报告一路上读到的颜色种类，供你决定终点色标用什么颜色。

安全：看到白色就后退转向，不会冲出边界；但仍需先把小车放在场地内部。
"""

import random

import config
import edge
from common import battery_check, log, wait_button_or_timeout
from hardware import color, drive, ev3

from pybricks.parameters import Color
from pybricks.tools import run_task, wait

MISSION_MAX_MS = 60000      # 整场最长运行时间
PUSH_MS = 1500              # 推目标物的时长
LOOP_MS = config.SAMPLE_PERIOD_MS


async def explore_step(threshold):
    """走一步并处理边界。返回 ("edge", turn_deg) 或 ("ok", None)。"""
    r = await color.reflection()
    if r > threshold and await edge.hit_edge(threshold):
        ev3.light.on(Color.RED)
        drive.stop()
        await wait(100)
        drive.straight(-60)
        turn = random.choice([-1, 1]) * random.choice([50, 80, 110, 140])
        drive.turn(turn)
        ev3.light.on(Color.GREEN)
        return "edge", turn
    return "ok", None


async def main():
    ev3.screen.clear()
    log(ev3, "=== mission ===")
    battery_check(ev3, config.LOW_BATTERY_MV)

    threshold = config.edge_threshold()
    log(ev3, "edge threshold", threshold)

    if config.FINISH_COLOR is None:
        log(ev3, "FINISH_COLOR not set -> explore only")

    await wait_button_or_timeout(ev3, 8000, prompt="RIGHT to start")
    ev3.light.on(Color.GREEN)

    seen = set()
    t = 0
    edges = 0
    pushes = 0
    state = "EXPLORE"
    push_left = 0

    while t < MISSION_MAX_MS:
        if state == "EXPLORE":
            c = await color.color()
            if c != Color.NONE:
                seen.add(c)

            if config.FINISH_COLOR is not None and c == config.FINISH_COLOR:
                drive.stop()
                log(ev3, "FINISH color detected!")
                state = "FINISH"
                break

            if (config.TARGET_COLOR is not None
                    and c == config.TARGET_COLOR):
                state = "PUSH"
                push_left = PUSH_MS
                log(ev3, "target color found -> push")
                ev3.speaker.beep(1000, 150)
                continue

            kind, turn = await explore_step(threshold)
            if kind == "edge":
                edges += 1
                log(ev3, "edge", edges, "turn", turn)
                continue

            drive.drive(config.EXPLORE_SPEED, 0)

        elif state == "PUSH":
            drive.drive(config.DRIVE_SPEED, 0)
            push_left -= LOOP_MS
            if push_left <= 0:
                drive.stop()
                pushes += 1
                log(ev3, "push done", pushes)
                state = "EXPLORE"
                drive.turn(90)

        await wait(LOOP_MS)
        t += LOOP_MS

    drive.stop()

    if state != "FINISH":
        log(ev3, "mission timeout")

    log(ev3, "edges:", edges, "pushes:", pushes)
    print("colors seen:", [str(c) for c in seen])

    ev3.speaker.beep(500, 600)
    log(ev3, "=== end ===")


run_task(main())
