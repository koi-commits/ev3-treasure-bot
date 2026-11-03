"""
config.py — 全局参数，single source of truth
Pybricks v4.1.2 / LEGO MINDSTORMS EV3

规则 (rules):
  * 所有标 [TBD] 的值都是占位符 (placeholder)，不是测量值。
  * 实物确认后：改数值 + 把对应的 *_CONFIRMED 改为 True + 记录到 docs/calibration.md
  * 其他脚本只 import 这里，不要在测试脚本里写死端口或尺寸。

2026-11-03 实机探测结果（src/diag.py）:
  * 电机在 Port.A 和 Port.D（B、C 空）
  * 颜色传感器在 Port.S1（S2/S3/S4 空）
  * 固件 4.1.2，电池 7477 mV
"""

from pybricks.parameters import Direction, Port

# ============================================================
# 1. Ports  —— 已由 src/diag.py 实机扫描确认
# ============================================================
LEFT_MOTOR_PORT = Port.A        # 确认：A = 左轮（2026-11-03 人工观察 motor_id.py）
RIGHT_MOTOR_PORT = Port.D       # 确认：D = 右轮
COLOR_SENSOR_PORT = Port.S1     # 确认
PORTS_CONFIRMED = True
LEFT_RIGHT_CONFIRMED = True

# ============================================================
# 2. Motor direction  —— 用 test_motors.py Phase 1 确认
#    目标：给"正速度"时，该轮让小车向前滚。
#    若某个轮子转反，只改这里，不要在代码里加负号。
# ============================================================
LEFT_MOTOR_DIRECTION = Direction.CLOCKWISE    # 确认：正速度朝车头滚
RIGHT_MOTOR_DIRECTION = Direction.CLOCKWISE   # 确认：正速度朝车头滚
DIRECTIONS_CONFIRMED = True

# 电机到轮子之间的齿轮组。None = 轮子直接装在电机输出轴上。
# 例：电机 12 齿带动轮轴 36 齿 -> [12, 36]
GEARS = None                    # [TBD]

# ============================================================
# 3. Geometry (mm)  —— 先用尺子量，再用 1000mm / 360° 实测修正
#    wheel_diameter: 走不够远 -> 调小；走过头 -> 调大
#    axle_track:     转不够   -> 调大；转过头 -> 调小
#    先调 wheel_diameter，再调 axle_track。
# ============================================================
WHEEL_DIAMETER_MM = 56.0        # [TBD] 占位，未测量
AXLE_TRACK_MM = 114.0           # [TBD] 占位，未测量（两轮触地点中心距）
GEOMETRY_CONFIRMED = False

# ============================================================
# 4. Test motion profile —— 阶段一刻意保守 (low speed first)
# ============================================================
TEST_WHEEL_SPEED = 180          # deg/s，单轮测试转速
TEST_STRAIGHT_SPEED = 100       # mm/s
TEST_STRAIGHT_ACCEL = 200       # mm/s^2
TEST_TURN_RATE = 60             # deg/s
TEST_TURN_ACCEL = 120           # deg/s^2
TEST_DISTANCE_MM = 200          # 直行/后退测试距离
TEST_TURN_DEG = 90              # 原地转向测试角度
TEST_DRIVE_TIME_MS = 2000       # drive() 连续行驶测试时长

# ============================================================
# 5. Color sensor
#    安装高度建议先试 5–10 mm 离地，需实测确认最稳定的高度。
# ============================================================
SENSOR_HEIGHT_MM = None         # [TBD] 实测后填写
SAMPLE_PERIOD_MS = 50           # 采样周期
STATS_WINDOW = 20               # 滑动窗口样本数
CAL_SAMPLES = 30                # 标定时每个表面取样数

BLACK_REFLECTION = 0.7          # 标定 n=30：min 0.9 max 1.1；场地实测 low20=0.7
WHITE_REFLECTION = 12.0         # 标定 n=30：11.2；场地实测 high20=11.5, max=14.5
COLOR_CONFIRMED = True
# 场地实测（src/probe_live.py，2026-11-03，手推过白边 30s）：
#   low20=0.7  high20=11.5  span=10.7  —— 黑白分离 ~16 倍，过渡在 0.5s 内完成（很陡）

# ============================================================
# 6. Safety
# ============================================================
LOW_BATTERY_MV = 7000           # 低于此电压给出警告；低电量会让速度/标定漂移

# ============================================================
# 7. Driving / line following  —— 阶段二
# ============================================================
DRIVE_SPEED = 120               # mm/s，巡线巡航速度
DRIVE_SLOW = 70                 # mm/s，靠近边界/找目标时的慢速
TURN_RATE = 120                 # deg/s，原地转

# ---- 触边转向式沿边巡线（bang-bang，实测更不容易冲出白边）----
# 实测 2026-11-03：纯比例控制在 70mm/s 下会"扫到白边→转向→仍冲出去"，
# 因为 drive(speed, steer) 保持前进速度，发现白色时车身已经深入白区。
# 改成：深色区朝白边小角度斜行；一旦见白立刻原地转向深色侧。
FOLLOW_SPEED = 60               # mm/s 沿边前进速度
APPROACH_STEER = 22             # deg/s 朝白边侧的偏转量（斜向接近边缘）
EDGE_TURN_RATE = 130            # deg/s 见白后原地转向速度
EDGE_ESCAPE_MS = 350            # 见白后**必定**原地转的时长（约 45°），不能提前结束
                                # 实测：只按"转到不白为止"会在内转角处来回抖，
                                # 因为刚转出去就立刻又斜着撞回来。
EDGE_CLEAR_MS = 400             # 逃逸后"盲走"的时长，此期间忽略传感器，防止抖振

# mission.py 用的巡线参数组（demo_edge.py 始终用上面的已验证参数）
# "validated"：11-03 实机通过的参数。车几乎平行于边时，边侧轮子会压上白带（仿真中最深约 36 mm）
# "steep"    ：同一状态机，更大逃逸角 + 更强回转，碰边入射角更陡，轮子压白更浅（仿真最深约 29 mm），
#              慢约 20%。【只在仿真中验证】白带窄（< 40 mm）或规则不允许压线时在实机 A/B 测试。
FOLLOW_PROFILE = "validated"
FOLLOW_PROFILES = {
    "validated": (22, 350, 400),     # (APPROACH_STEER deg/s, EDGE_ESCAPE_MS, EDGE_CLEAR_MS)
    "steep": (50, 600, 300),
}

# 巡线增益（比例控制，误差按 black/white 对比度归一化）
KP_NORM = 1.6                   # 归一化误差 -> 转向量比例
STEER_LIMIT = 70                # 转向量限幅 (deg/s)
EDGE_MARGIN = 6                 # 触发转向的反射率余量 (%)

# 白色边界：反射率高于阈值视为"白"（桌子外/边界线）
# 阈值由 test_color.py 标定，未标定时用 DEFAULT_EDGE_THRESHOLD
DEFAULT_EDGE_THRESHOLD = 45.0
EDGE_CONFIRM_SAMPLES = 2        # 连续几个样本超过阈值才认定为边界
                                # 50ms x 2 = 100ms；70mm/s 时约 7mm 越界量，可接受
                                # （场地过渡很陡，3 个样本会多冲 3.5mm，改 2）

# ============================================================
# 8. Mission  —— 阶段三 自主寻宝
# ============================================================
# 旧版 mission 参数（2026-11-03 晚（离线开发）起 mission.py 不再使用，保留供参考）
EXPLORE_SPEED = 130
EXPLORE_TURN_MS = 900
SEARCH_SPIN_DEG = 45

# ============================================================
# 9. 传感模式与色标  —— 2026-11-03 晚新增，全部【待实机验证】
#    流程：probe_timing.py -> color_probe.py -> calibrate_marker.py -> 改这里
# ============================================================
# "reflection" = 已验证基线（只判白边，无色标）
# "rgb"        = 单模式 RGB：白边 + 色标（需 SIGNATURES 与 RGB_EDGE_BRIGHT）
# "dual"       = reflection 判边 + rgb 判色标（每 tick 切两次模式，慎用）
SENSE_MODE = "reflection"

RGB_EDGE_BRIGHT = None          # [TBD] rgb 模式下"亮度 > 此值且未识别"视为白边
MARKER_CONFIRM = 3              # 色标须连续 3 个样本（150 ms）同类才确认
FINISH_LABEL = "finish"         # SIGNATURES 中终点色标的名字

# calibrate_marker.py 会打印一段可直接粘贴的 SIGNATURES。
# 特征顺序：(r, g, b, bright, cr, cg)；tol 为 None 的维度不检查。
SIGNATURES = {}                 # [TBD]

# ============================================================
# 10. 自主任务（mission.py）—— 全部【待实机验证】
# ============================================================
# "edge_to_finish" : 寻边 -> 沿边巡线 -> 遇终点色标停车     （下周二主测）
# "survey"         : 寻边 -> 沿边巡线一段时间，只记录色标与遥测，不停车
# "calib"          : 找终点 -> 色标定位 -> 场上转角自标定（打印 turn_k），不需要物体
# "capture"        : 只做搜索 + 负载嫌疑 + tap 确认（不放物体时测误报率）
# "full"           : 找终点 -> 自标定 -> 随机反弹搜索 -> tap 确认 -> 推送 -> 交付
MISSION_MODE = "edge_to_finish"
MISSION_MAX_MS = 600000         # 整场兜底（full 模式需要数分钟）
START_TIMEOUT_MS = 8000         # 等 RIGHT 键，超时自动开始（便于 pbrun 无人值守）

EDGE_SIDE = -1                  # 白边在车左侧（与已验证 demo_edge 一致）

# 寻边（随机起点/朝向）
SEEK_SPEED = 90                 # mm/s，与 demo_edge 找边速度一致
SEEK_ATTEMPTS = 6
SEEK_BACKOFF_MM = 30            # 找到边后后退量（demo_edge 的未 await 后退实际更短）
START_BACK_MAX_MM = 200         # 起步光斑就在白边上：最多倒车这么远找回地面

# 巡线监督 / 异常恢复
FOLLOW_MAX_MS = 120000          # 沿边找终点的上限（应 > 绕场一圈时间，[TBD]）
LOST_EDGE_MS = 6000             # CRUISE 中这么久没见白 -> 认为丢边，重新直行寻边
OVERRUN_BACK_MM = 60            # 续转用完仍在白区 -> 先倒车这么多
OVERRUN_LIMIT = 4               # 连续越界恢复次数上限，超过则中止
STALL_MS = 600                  # drive.stalled() 持续这么久 -> 撞到东西，后退转开
STALL_BACK_MM = 80
STALL_TURN_DEG = 60

# 出界判定：凸（矩形）场地内，任何不见白的直线段都不可能长于对角线。
# 直行超过 ARENA_DIAG_MM 仍未见白 -> 判定已在场外，朝最近见白位置返回并穿过白带。
ARENA_DIAG_MM = 1600            # [TBD] 实测场地内部对角线 × 1.1（偏大只会恢复变慢，偏小会误判）
RETURN_IN_MM = 150              # 穿过白带回到场内后再前进这么多

# 到达终点 / 放下物体
FINISH_BACK_MM = 100            # 送达后倒车离开（V 形推板倒车 = 把物体留在原地）

# 几何 —— 推送与交付要用（[TBD] 实机量）
SENSOR_OFFSET_MM = 60           # [TBD] 光斑中心到两轮轴线中点的前向距离（M1）
OBJ_LEAD_MM = 110               # [TBD] 物体卡在推板顶点时，物体中心到两轮轴线中点的前向距离（M8）

# 搜索：随机反弹覆盖（直行到边 -> 退 -> 随机转 100–170°）。不依赖几何标定。
# （往返扫描线方案在仿真中成功率更低且有出界，已删除，见 docs/offline-2026-11-03.md）
BOUNCE_MAX_LEGS = 60
SWEEP_SPEED = 100
SWEEP_END_BACK_MM = 90          # [TBD] 到边后原地转之前的倒车量，须 ≥ 光斑前伸距离 + 20

# 推到物体判定：负载嫌疑（LoadMonitor）+ tap 确认
CAPTURE_MODE = "load"           # "load" 或 "off"（off = 只扫场不判定，无法完成交付）
LOAD_SKIP_N = 6                 # 每段直行前 6 个样本是加速段
LOAD_LEARN_N = 12               # 每段前 12 个样本参与基线（基线只往下更新）
LOAD_WINDOW = 5
LOAD_K = 4.0
LOAD_ABS_MARGIN = 15.0          # [TBD] mNm，probe_load.py 实测后改
LOAD_HOLD_N = 8                 # 连续 8 个样本（≈40 mm）超阈值才算嫌疑
TAP_BACK_MM = 50                # tap：倒车松开物体的距离
TAP_SPEED = 60                  # tap：重新前进速度（mm/s）
TAP_MARGIN = 8.0                # [TBD] mNm，重新接触段比自由段高出这么多才确认
TAP_SEAT_MM = 120               # tap 重新接触后再直推这么远：晚段负载仍高 = 物体在口袋里（不是只蹭到臂外侧）
CARRY_LOST_N = 12               # 推送中负载连续 12 个样本回落到空载水平 -> 判定物体丢失

# 色标定位与交付几何
MAP_MS = 9000                   # 首次确认终点后继续沿边走这么久，求色标中心与场地轴向
MARKER_INSET_MM = 15            # [TBD] 目标点 = 色标中心估计再往场内移这么多（色标深度/4 左右）
APPROACH_STANDOFF_MM = 250      # 交付前先到目标点正前方（场内侧）这么远的待命点，再直线推进
# 场上转角自标定（full 模式，记录终点后在同一条边上做一次，约 20 s）
SELF_CAL = True
SELFCAL_IN_MM = 160             # 退到离边这么远再原地转（光斑转圈不会碰到白边）
SELFCAL_SPIN = 720              # 原地转的角度（越大越准，误差 ∝ 1/转角）
SELFCAL_FOLLOW_MS = 9000        # 回到边上沿边走多久来测边方向
SELFCAL_MAX_ERR = 60            # 测得误差超过这个（度）视为测量失败，不修正

EDGE_PEEK = False               # 【实验，默认关】碰边后沿边走一小段测边方向、修正里程计航向。
                                # 仿真：每次进入巡线都有小概率轮子越线，十几段累积后几乎必然越线 -> 不启用
PEEK_MS = 4000
PEEK_MAX_CORR = 25              # 单次修正上限（度）；超过说明测量不可信，不修正
BOUNCE_JITTER_DEG = 15          # 随机反弹：对角线方向 ± 抖动（入射角 30–60°）

# 推送模式（carrying）
CARRY_SPEED = 70                # mm/s
CARRY_TURN_RATE = 35            # deg/s，弧线转向（半径约 CARRY_SPEED/rad(rate) ≈ 115 mm）
PURSUIT_GAIN = 1.5              # 纯追踪：转向率(deg/s) = 增益 × 相对方位角(deg)，限幅 CARRY_TURN_RATE
CARRY_ESCAPE_SPEED = 40         # mm/s，推送中沿边 ESCAPE 用前进急弯（0 = 已验证的原地转）


# ------------------------------------------------------------
def black_white_threshold():
    """黑白阈值 = 两者中点。未标定时返回 None，调用方必须处理。"""
    if BLACK_REFLECTION is None or WHITE_REFLECTION is None:
        return None
    return (BLACK_REFLECTION + WHITE_REFLECTION) / 2


def edge_threshold():
    """边界阈值：优先用标定值，否则退回默认值。"""
    t = black_white_threshold()
    return t if t is not None else DEFAULT_EDGE_THRESHOLD


def unconfirmed_items():
    """返回还没在实物上确认的参数组名称，测试脚本启动时会打印出来。"""
    items = []
    if not PORTS_CONFIRMED:
        items.append("ports")
    if not LEFT_RIGHT_CONFIRMED:
        items.append("left_right")
    if not DIRECTIONS_CONFIRMED:
        items.append("directions")
    if not GEOMETRY_CONFIRMED:
        items.append("geometry")
    if not COLOR_CONFIRMED:
        items.append("color")
    return items
