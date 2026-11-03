r"""
diag.py — 现场诊断：固件/电池/端口/传感器。只读，不驱动电机。

用 pb CLI 直接跑：
    pbrun.cmd src\diag.py

关键约束（Pybricks 4.x）：Motor / ColorSensor 这类设备对象必须在模块顶层构造，
也就是在 run_task() 启动多任务之前。在 async 函数里 new 一个设备会抛
RuntimeError: "This can only be called before multitasking starts."
"""

from pybricks.ev3devices import ColorSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import run_task

ev3 = EV3Brick()

MOTOR_PORT_NAMES = ("A", "B", "C", "D")
SENSOR_PORT_NAMES = ("S1", "S2", "S3", "S4")


def why(e):
    """MicroPython 的异常类型没有 __name__，用 repr 描述。"""
    return repr(e)


# ---------------------------------------------------------------
# 设备探测：必须在模块级完成
# ---------------------------------------------------------------
motors = {}
for _name in MOTOR_PORT_NAMES:
    try:
        motors[_name] = Motor(getattr(Port, _name))
    except Exception as _e:  # noqa: BLE001 - 端口空着就会抛 OSError
        motors[_name] = None

colors = {}
for _name in SENSOR_PORT_NAMES:
    try:
        colors[_name] = ColorSensor(getattr(Port, _name))
    except Exception as _e:  # noqa: BLE001
        colors[_name] = None


async def main():
    print("=== system ===")
    try:
        info = ev3.system.info()
        for k in sorted(info):
            print("  {} = {!r}".format(k, info[k]))
    except Exception as e:
        print("  system.info() FAILED:", why(e))

    print("=== battery ===")
    print("  voltage mV:", ev3.battery.voltage())
    print("  current mA:", ev3.battery.current())

    print("=== motor ports ===")
    for name in MOTOR_PORT_NAMES:
        m = motors[name]
        if m is None:
            print("  {}: -".format(name))
        else:
            print("  {}: MOTOR ok  angle={}".format(name, m.angle()))

    print("=== sensor ports ===")
    for name in SENSOR_PORT_NAMES:
        s = colors[name]
        if s is None:
            print("  {}: -".format(name))
        else:
            try:
                r = await s.reflection()
                c = await s.color()
                print("  {}: COLOR ok  reflection={} color={}".format(name, r, c))
            except Exception as e:
                print("  {}: COLOR error {}".format(name, why(e)))

    print("=== done ===")


run_task(main())
