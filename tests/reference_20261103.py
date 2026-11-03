"""11-03 实机验证版 follow_edge_bangbang 的状态机部分，原样保留作为对拍基准（oracle）。
唯一改动：去掉 await / 硬件，改为对一串反射率同步运行，记录 drive 命令序列。"""


def follow_edge_bangbang_reference(refls, threshold, speed, side, approach_steer,
                                   turn_rate, escape_ms, clear_ms, period):
    away = -side * turn_rate          # 离开白边的转向（原地）
    toward = side * approach_steer    # 朝白边的偏转（前进中）

    CRUISE, ESCAPE, CLEAR = 0, 1, 2

    cmds = []
    t = 0
    state = CRUISE
    until = 0
    extensions = 0
    twists = 0

    for refl in refls:
        if state == CRUISE:
            if refl > threshold:
                twists += 1
                state = ESCAPE
                until = t + escape_ms
                extensions = 0
                cmds.append((0, away))
            else:
                cmds.append((speed, toward))

        elif state == ESCAPE:
            cmds.append((0, away))
            if t >= until:
                if refl > threshold and extensions < 2:
                    extensions += 1
                    until = t + escape_ms
                else:
                    state = CLEAR
                    until = t + clear_ms

        else:  # CLEAR
            cmds.append((speed, toward))
            if t >= until:
                state = CRUISE

        t += period
    return cmds, twists
