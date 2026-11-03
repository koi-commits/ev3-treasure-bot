r"""
probe_sensor.py — 只读：连续采样颜色传感器，看实物到底返回什么。

    pbrun.cmd src\probe_sensor.py

不驱动电机，机器人可以放在任何位置。
"""

from pybricks.ev3devices import ColorSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import run_task, wait

ev3 = EV3Brick()
sensor = ColorSensor(Port.S1)


async def main():
    print("sampling S1 for 10 samples ...")
    for i in range(10):
        r = await sensor.reflection()
        c = await sensor.color()
        amb = await sensor.ambient()
        rgb = await sensor.rgb()
        print("{:2d}  refl={:.1f}  amb={}  rgb={}  color={}".format(
            i, r, amb, rgb, c))
        await wait(300)
    print("done")


run_task(main())
