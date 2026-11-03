"""
check_env.py — Step 0：确认固件、电池、端口上接了什么
只读检查：不驱动电机、不写存储、不改固件。

Expected：屏幕/终端列出 A–D 哪些口有电机，S1–S4 哪些口有颜色传感器。
用结果去改 config.py 第 1 节，然后把 PORTS_CONFIRMED 设为 True。
"""

from pybricks.ev3devices import ColorSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import run_task, wait

ev3 = EV3Brick()

MOTOR_PORTS = (Port.A, Port.B, Port.C, Port.D)
SENSOR_PORTS = (Port.S1, Port.S2, Port.S3, Port.S4)


def probe_motor(port):
    try:
        m = Motor(port, reset_angle=False)
        m.close()          # 释放端口，避免影响后续脚本
        return True
    except OSError:
        return False


async def probe_color(port):
    try:
        s = ColorSensor(port)
        r = await s.reflection()
        return r
    except OSError:
        return None


async def main():
    ev3.screen.clear()
    info = ev3.system.info()
    print("System info:", info)
    ev3.screen.print("Name:", info.get("name"))
    ev3.screen.print("Batt:", ev3.battery.voltage(), "mV")

    print("--- Motor ports ---")
    for p in MOTOR_PORTS:
        ok = probe_motor(p)
        line = "{} motor".format(p) if ok else "{} -".format(p)
        print(line)
        ev3.screen.print(line)

    print("--- Sensor ports (ColorSensor) ---")
    for p in SENSOR_PORTS:
        r = await probe_color(p)
        line = "{} color r={}".format(p, r) if r is not None else "{} -".format(p)
        print(line)
        ev3.screen.print(line)

    print("Done. Update config.py section 1.")
    await wait(10000)   # 留 10 秒看屏幕


run_task(main())
