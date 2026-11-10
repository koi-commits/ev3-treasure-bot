"""urandom.py — CPython 替身（MicroPython 的 urandom 模块）。

为什么需要这个文件：
    src/mission.py 里写的是

        try:
            import urandom as random
        except ImportError:
            import random

    MicroPython 上 urandom 就是随机数模块。但本机 site-packages 里有一个
    Pybricks 的类型存根包也叫 `urandom`（D:\\python\\Lib\\site-packages\\urandom），
    它的函数全是空实现，`randint()` 返回 None。于是离线测试里 mission.py
    拿到的是空实现，表现为

        TypeError: list indices must be integers or slices, not NoneType
        TypeError: unsupported operand type(s) for *: 'NoneType' and 'int'

    tests/mock 在 sys.path 最前面，用这个文件遮蔽存根包，把行为对齐 MicroPython。
    这只影响 PC 离线测试，不影响 EV3 上的真实 urandom。

实现直接委托给 CPython 的 random，因此 tests/harness.py 的 random.seed(seed)
依然能保证可复现。
"""

import random as _random

# MicroPython urandom 的公开接口：getrandbits / seed / choice / randint /
# randrange / uniform。本文件额外提供 random()，因为代码里可能用到。


def seed(a=None):
    _random.seed(a)


def getrandbits(n):
    return _random.getrandbits(n)


def randrange(start, stop=None):
    if stop is None:
        return _random.randrange(start)
    return _random.randrange(start, stop)


def randint(a, b):
    return _random.randint(a, b)


def choice(seq):
    return _random.choice(seq)


def uniform(a, b):
    return _random.uniform(a, b)


def random():
    return _random.random()
