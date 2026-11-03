r"""
edge.py — 白色边界检测 + 沿边巡线。阶段二的核心算法，纯逻辑，不含 mission 流程。

概念：
  * 机器人站在"场地"（深色）上，场地边缘/边界线是"白色"。
  * 颜色传感器朝下，反射率 reflection 0–100。
  * refl > threshold  => 看到白色（边界）
  * refl < threshold  => 看到深色（场地）

用法：
  1. read_reflection()            单次读数
  2. hit_edge(threshold)          连续 N 个样本都白 -> 认定到边界（防抖）
  3. follow_edge_bangbang(...)    ★ 已实机验证的三状态沿边巡线（逻辑在 fsm.EdgeFollower）
  4. avoid_edge(...)              撞边自保：后退 + 转向
  5. follow_edge(...)             比例控制版本，实测冲出白边，仅留作对照

所有函数都是 async，配合 run_task 使用。
"""

import config
from fsm import EdgeFollower
from hardware import color, drive

from pybricks.tools import wait


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


async def read_reflection():
    return await color.reflection()


async def read_rgb():
    return await color.rgb()


async def hit_edge(threshold=None, samples=None):
    """连续 samples 次读数都超过阈值才算到边界，避免单个噪声点误判。"""
    if threshold is None:
        threshold = config.edge_threshold()
    if samples is None:
        samples = config.EDGE_CONFIRM_SAMPLES
    hits = 0
    for _ in range(samples):
        r = await color.reflection()
        if r > threshold:
            hits += 1
        await wait(config.SAMPLE_PERIOD_MS)
    return hits >= samples


async def avoid_edge(threshold=None, back_mm=70, turn_deg=90):
    """发现白边后的自保动作：先退，再转开。返回转过的角度。"""
    drive.stop()
    await wait(150)
    # 2026-11-03 晚修复：原代码未 await，turn 会立刻覆盖 straight，倒车根本不发生
    await drive.straight(-abs(back_mm))
    await drive.turn(turn_deg)
    return turn_deg


def contrast_span():
    """黑白反射率差，用来把误差归一化，适应任意标定区间。"""
    b = config.BLACK_REFLECTION
    w = config.WHITE_REFLECTION
    if b is None or w is None:
        return 20.0
    span = w - b
    return span if span > 5.0 else 5.0


async def follow_edge(threshold=None, speed=None, gain=None, side=-1,
                      max_ms=8000, on_tick=None):
    """沿边巡线（归一化比例控制）。

    参数：
      threshold : 边界阈值（反射率）
      speed     : 巡航速度 mm/s
      gain      : 归一化增益，误差/对比度 -> 转向比例
      side      : 白线在小车的哪一侧。+1 = 右侧，-1 = 左侧。
                  看到比期望更多的白色时，朝反方向转，把白线"推"回视野边缘。
      max_ms    : 最长运行时间，兜底防止跑飞
    """
    if threshold is None:
        threshold = config.edge_threshold()
    if speed is None:
        speed = config.DRIVE_SPEED
    if gain is None:
        gain = config.KP_NORM
    span = contrast_span()

    drive.reset()
    t = 0
    period = config.SAMPLE_PERIOD_MS
    while t < max_ms:
        refl = await color.reflection()
        err = (refl - threshold) / span      # -1 .. +1 左右
        steer = clamp(-side * gain * err * config.STEER_LIMIT,
                      -config.STEER_LIMIT, config.STEER_LIMIT)
        drive.drive(speed, steer)
        if on_tick is not None:
            on_tick(t, refl, steer)
        await wait(period)
        t += period

    drive.stop()
    return drive.distance()


async def follow_edge_bangbang(threshold=None, speed=None, side=-1, max_ms=8000,
                               approach_steer=None, turn_rate=None,
                               escape_ms=None, clear_ms=None, on_tick=None):
    """触边转向式沿边巡线（bang-bang + 定时保险）。

    三个状态循环：
      CRUISE : 向前 + 朝白边偏 approach_steer，斜向逼近边界
      ESCAPE : 一见到白就**原地**转 away 方向，固定 escape_ms（约 45°），
               不再前进；若转完仍在白区，最多再续 2 次
      CLEAR  : 逃逸后盲走 clear_ms，此期间**忽略传感器**，避免刚转出去就
               立刻又斜着撞回来（实测这样会在内转角处来回抖）

    为什么不用纯比例控制：`drive.drive(speed, steer)` 保持前进速度，
    等传感器读到白色时车头已经深入白区，比例项虽满舵但仍在向前推进，
    实测会"扫到白边→有转向→照样冲出白边"。

    side: 白线在小车哪一侧，+1 = 右侧，-1 = 左侧。
    返回触发逃逸的次数（沿边锯齿数）。
    """
    if threshold is None:
        threshold = config.edge_threshold()
    if speed is None:
        speed = config.FOLLOW_SPEED
    if approach_steer is None:
        approach_steer = config.APPROACH_STEER
    if turn_rate is None:
        turn_rate = config.EDGE_TURN_RATE
    if escape_ms is None:
        escape_ms = config.EDGE_ESCAPE_MS
    if clear_ms is None:
        clear_ms = config.EDGE_CLEAR_MS

    period = config.SAMPLE_PERIOD_MS
    # 2026-11-03 晚：状态机逻辑移到 fsm.EdgeFollower（纯逻辑，可离线对拍）。
    # tests/test_fsm.py 用 11-03 原实现逐 tick 对比，drive 命令序列完全一致。
    f = EdgeFollower(side, speed, approach_steer, turn_rate, escape_ms, clear_ms)

    drive.reset()
    t = 0
    while t < max_ms:
        refl = await color.reflection()
        sp, steer = f.step(t, refl > threshold)
        drive.drive(sp, steer)
        if on_tick is not None:
            on_tick(t, refl, steer)
        await wait(period)
        t += period

    drive.stop()
    return f.twists
