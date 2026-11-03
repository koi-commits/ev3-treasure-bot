from sim import SIM_HOLDER


class _Sleep:
    def __init__(self, ms):
        self.ms = ms

    def __await__(self):
        yield ("sleep", self.ms)

    __iter__ = __await__   # MicroPython 的 await 走迭代器协议


def wait(ms):
    return _Sleep(ms)


class StopWatch:
    def __init__(self):
        self._t0 = SIM_HOLDER.sim.t

    def time(self):
        return int(SIM_HOLDER.sim.t - self._t0)

    def reset(self):
        self._t0 = SIM_HOLDER.sim.t


def run_task(coro):
    SIM_HOLDER.sim.run(coro)


async def multitask(*coros, race=False):
    raise NotImplementedError("multitask not mocked")
