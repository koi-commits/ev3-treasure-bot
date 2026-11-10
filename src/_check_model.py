"""_check_model.py — 在 EV3 本体上验证 ContactGRU(tap_probe) 权重可加载、推理可运行。

用途：回答"模型到底有没有部署到 EV3 上"，用 hub 自己打印的证据，而不是 PC 上跑通。

做法：
  1. 在 hub 上 import tap_probe_weights（MicroPython 导出）与 contact_inference；
  2. 打印模型元信息（INPUT_SIZE / HIDDEN_SIZE / THRESHOLD / DEBOUNCE_N / 参数量）；
  3. 用一段**确定性**特征序列（LCG 生成，PC 端可复现）跑 40 步 GRU，
     打印概率与是否越过阈值；
  4. 打印剩余内存，确认纯 Python 推理能在这块板子上跑。

PC 端同序列复现：D:\\python\\python.exe tools\\model_parity.py，逐位对比概率。

不涉及任何电机运动。按 RIGHT 开始，或 10 秒后自动开始。
"""

import gc

import config
import tap_probe_weights as weights
from common import log
from contact_features import tap_probe_feature
from contact_inference import ContactInference
from hardware import ev3
from pybricks.parameters import Button, Color
from pybricks.tools import StopWatch, run_task, wait

STEPS = 40
PRINT_EVERY = 5


def lcg(state):
    """与 PC 端一致的小随机源。

    注意：EV3 的 MicroPython 只有 30 位小整数，1103515245 这种乘子会抛
    `OverflowError: long int not supported in this build`，所以特地用小模数
    （65521 是费马素数），保证每一步都留在小整数范围内。
    """
    return (state * 1103 + 12345) % 65521


def features_for(i, state):
    """用 i 和 LCG 造一组物理上大致合理的特征。"""
    state = lcg(state)
    load_l = ((state >> 8) % 400) - 200
    state = lcg(state)
    load_r = ((state >> 8) % 400) - 200
    state = lcg(state)
    dist_step = ((state >> 8) % 60) / 10.0
    state = lcg(state)
    progress = ((state >> 8) % 120) / 100.0
    phase = i % 7
    return tap_probe_feature(load_l, load_r, dist_step, progress,
                             is_free=(phase == 0),
                             is_early=(phase == 1 or phase == 2),
                             is_late=(phase == 5 or phase == 6),
                             probe_left=(phase == 3),
                             probe_right=(phase == 4)), state


async def main():
    log(ev3, "MODEL CHECK")
    print("MODEL_META name=tap_probe_gru_v1 input={} hidden={} threshold={} debounce_n={}".format(
        weights.INPUT_SIZE, weights.HIDDEN_SIZE, weights.THRESHOLD, weights.DEBOUNCE_N))
    params = (len(weights.W_IH) * len(weights.W_IH[0])
              + len(weights.W_HH) * len(weights.W_HH[0])
              + len(weights.B_IH) + len(weights.B_HH)
              + len(weights.W_OUT) + 1)
    print("MODEL_PARAMS total={} w_ih={}x{} w_hh={}x{} w_out={}".format(
        params, len(weights.W_IH), len(weights.W_IH[0]),
        len(weights.W_HH), len(weights.W_HH[0]), len(weights.W_OUT)))
    print("MODEL_MEM mem_free_before={}".format(gc.mem_free()))

    await wait_start()

    net = ContactInference(weights)
    sw = StopWatch()
    state = 12345
    hits = 0
    maxp = 0.0
    for i in range(STEPS):
        feat, state = features_for(i, state)
        prob, fired = net.update(feat)
        if prob > maxp:
            maxp = prob
        if fired:
            hits += 1
        if i % PRINT_EVERY == 0 or i == STEPS - 1:
            print("MODEL_STEP i={:02d} p={:.6f} fired={} run={} t_ms={}".format(
                i, prob, int(fired), net.run, sw.time()))
        await wait(config.SAMPLE_PERIOD_MS)
    print("MODEL_DONE steps={} time_ms={} max_p={:.6f} fired_count={} mem_free_after={}".format(
        STEPS, sw.time(), maxp, hits, gc.mem_free()))
    log(ev3, "MODEL DONE")


async def wait_start():
    """按 RIGHT 或 10 秒超时（无人值守也能跑完）。"""
    sw = StopWatch()
    ev3.light.on(Color.YELLOW)
    while sw.time() < 10000:
        if Button.RIGHT in ev3.buttons.pressed():
            break
        await wait(50)
    ev3.light.on(Color.GREEN)


run_task(main())
