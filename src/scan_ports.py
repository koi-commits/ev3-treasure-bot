r"""
scan_ports.py — 扫描全部端口，识别插了什么设备。

    pbrun.cmd src\scan_ports.py

对每个端口依次尝试构造各种设备类，构造成功的就是识别到的类型。
同时打印颜色传感器的原始读数（rgb / ambient / reflection / color），
用来判断传感器到底是"没插好"还是"读不到光"。

注意：设备必须在模块顶层构造（Pybricks 4.x 限制），所以这里在
import 阶段就把所有能构造的设备建好，main 里只做打印和读取。
"""

from pybricks.ev3devices import (
    ColorSensor,
    GyroSensor,
    InfraredSensor,
    Motor,
    TouchSensor,
    UltrasonicSensor,
)
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import run_task, wait

ev3 = EV3Brick()

PORT_NAMES = ("S1", "S2", "S3", "S4", "A", "B", "C", "D")
CLASSES = (
    ("Motor", Motor),
    ("Touch", TouchSensor),
    ("Color", ColorSensor),
    ("Ultrasonic", UltrasonicSensor),
    ("Gyro", GyroSensor),
    ("Infrared", InfraredSensor),
)

found = {}      # port_name -> (type_name, instance)
for _pname in PORT_NAMES:
    _port = getattr(Port, _pname)
    for _tname, _cls in CLASSES:
        try:
            _dev = _cls(_port)
        except Exception:
            continue
        found[_pname] = (_tname, _dev)
        break


async def main():
    print("=== port scan ===")
    for name in PORT_NAMES:
        if name in found:
            print("  {:2s}: {}".format(name, found[name][0]))
        else:
            print("  {:2s}: (empty)".format(name))

    print("=== color sensor raw ===")
    for name in PORT_NAMES:
        item = found.get(name)
        if not item or item[0] != "Color":
            continue
        s = item[1]
        print("  port", name)
        for i in range(4):
            refl = await s.reflection()
            amb = await s.ambient()
            rgb = await s.rgb()
            c = await s.color()
            try:
                hsv = await s.hsv()
            except Exception as e:
                hsv = repr(e)
            print("    refl={:6.1f} amb={:6.1f} rgb={} hsv={} color={}".format(
                refl, amb, rgb, hsv, c))
            await wait(250)
        try:
            print("    detectable:", s.detectable_colors)
        except Exception as e:
            print("    detectable: n/a", repr(e))

    print("=== done ===")


run_task(main())
