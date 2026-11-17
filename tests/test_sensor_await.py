"""硬件 v3 回归：触感 / 陀螺仪的读值必须经 common.aread()，mock 必须同构。

为什么要有这个测试：
  11-10 的教训就是"PC 上全绿、实机上 await 用错"。这批新传感器的读值到底是数字还是
  awaitable 还没实机确认（docs/calibration.md 里标 [TBD - confirm on hardware]）。
  mock 如果返回裸数字，"忘了 await"在 PC 上永远不会暴露 —— 所以这里做两件事：
    1) 静态扫描 src/*.py：`touch.pressed()` / `gyro.angle()` / `gyro.speed()` 只能出现在
       `aread(...)` 里或 `await` 后面（个别例外行用注释 `aread-ok:` 标出，比如 type() 探测）；
    2) 断言 mock 真的返回 awaitable（sim.Value），而不是 bool/float。
"""

import io
import os
import tokenize
import unittest

import harness  # noqa: F401  # 副作用：把 src / tests / tests/mock 插进 sys.path

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "src")

READ_CALLS = ("touch.pressed(", "gyro.angle(", "gyro.speed(")
OK_PREFIXES = ("aread(", "await")


def _clean_lines(path):
    """把注释与字符串字面量抹成空格（行号/列位置不变），返回 [(cleaned, raw)]。

    这样 docstring 里写的 `gyro.angle()` 示例不会被误判成真实调用。
    """
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()
    raw_lines = src.splitlines()
    chars = [list(line) for line in raw_lines]
    try:
        tokens = list(tokenize.generate_tokens(io.StringIO(src).readline))
    except (tokenize.TokenError, IndentationError):
        return [(line, line) for line in raw_lines]
    for tok in tokens:
        if tok.type not in (tokenize.COMMENT, tokenize.STRING):
            continue
        (sr, sc), (er, ec) = tok.start, tok.end
        for row in range(sr, er + 1):
            if row - 1 >= len(chars):
                break
            line = chars[row - 1]
            c0 = sc if row == sr else 0
            c1 = ec if row == er else len(line)
            for col in range(c0, min(c1, len(line))):
                line[col] = " "
    return [("".join(cleaned), raw) for cleaned, raw in zip(chars, raw_lines)]


class TestAwaitableSensorReads(unittest.TestCase):
    def test_src_reads_go_through_aread(self):
        offenders = []
        for name in sorted(os.listdir(SRC)):
            if not name.endswith(".py"):
                continue
            rows = _clean_lines(os.path.join(SRC, name))
            for index, (clean, raw) in enumerate(rows):
                prev_raw = rows[index - 1][1] if index else ""
                if "aread-ok" in raw or "aread-ok" in prev_raw:
                    continue
                for call in READ_CALLS:
                    start = 0
                    while True:
                        at = clean.find(call, start)
                        if at < 0:
                            break
                        start = at + 1
                        if clean[:at].rstrip().endswith(OK_PREFIXES):
                            continue
                        offenders.append("%s:%d  %s" % (name, index + 1, raw.strip()))
        self.assertEqual(
            [],
            offenders,
            "触感/陀螺读值必须写成 `await aread(x)`（数字与 awaitable 都兼容）：\n"
            + "\n".join(offenders),
        )

    def test_mocks_return_awaitables(self):
        from pybricks.ev3devices import GyroSensor, TouchSensor
        from pybricks.parameters import Port
        from sim import Value

        self.assertIsInstance(TouchSensor(Port.S1).pressed(), Value,
                              "mock TouchSensor.pressed() 必须返回 awaitable")
        gyro = GyroSensor(Port.S2)
        self.assertIsInstance(gyro.angle(), Value,
                              "mock GyroSensor.angle() 必须返回 awaitable")
        self.assertIsInstance(gyro.speed(), Value,
                              "mock GyroSensor.speed() 必须返回 awaitable")


if __name__ == "__main__":
    unittest.main()
