r"""
probe_lights.py — 只读：验证颜色传感器的补光灯是否工作。

    pbrun.cmd src\probe_lights.py

分别关灯/开灯采样。若开灯后 rgb 明显变大，说明传感器硬件正常，
只是当前没有反光面（需要把机器人放到白纸上）。
"""

from pybricks.ev3devices import ColorSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import run_task, wait

ev3 = EV3Brick()
sensor = ColorSensor(Port.S1)


async def sample(tag, n):
    print("--- " + tag + " ---")
    for i in range(n):
        r = await sensor.reflection()
        amb = await sensor.ambient()
        rgb = await sensor.rgb()
        print("  {}  refl={:.1f} amb={} rgb={}".format(i, r, amb, rgb))
        await wait(250)


async def main():
    try:
        sensor.lights_off()
    except Exception as e:
        print("lights_off not available:", repr(e))
    await wait(300)
    await sample("lights OFF", 3)

    try:
        sensor.lights_on()
    except Exception as e:
        print("lights_on not available:", repr(e))
    await wait(300)
    await sample("lights ON", 8)
    print("done")


run_task(main())
