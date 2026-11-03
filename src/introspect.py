"""introspect.py — 现场探查 Pybricks 4.1.2 EV3 上真实可用的 API。只读。"""

from pybricks.hubs import EV3Brick
from pybricks.tools import run_task


def short(x):
    try:
        return "{!r}".format(x)
    except Exception:
        return "<unprintable>"


async def main():
    ev3 = EV3Brick()

    print("=== dir(ev3) ===")
    print(", ".join(sorted(n for n in dir(ev3) if not n.startswith("__"))))

    print("=== ev3.system ===")
    sysm = ev3.system
    print("type:", short(sysm))
    print("dir:", ", ".join(sorted(n for n in dir(sysm) if not n.startswith("__"))))
    for n in ("name", "version", "bootloader", "board_id"):
        try:
            attr = getattr(sysm, n)
            val = attr() if callable(attr) else attr
            print("  {} = {}".format(n, short(val)))
        except Exception as e:
            print("  {} FAILED: {}".format(n, short(e)))
    try:
        print("  info() =", short(sysm.info()))
    except Exception as e:
        print("  info() FAILED:", short(e))

    print("=== ev3.battery ===")
    print("dir:", ", ".join(sorted(n for n in dir(ev3.battery) if not n.startswith("__"))))

    print("=== ev3.screen ===")
    print("dir:", ", ".join(sorted(n for n in dir(ev3.screen) if not n.startswith("__"))))

    print("=== ports present ===")
    try:
        from pybricks.parameters import Port

        print("Port members:", short([p for p in dir(Port) if not p.startswith("_")]))
    except Exception as e:
        print("port dir failed:", short(e))

    print("=== done ===")


run_task(main())
