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
EXPLORE_SPEED = 130             # mm/s 探索速度
EXPLORE_TURN_MS = 900           # 撞边后随机转向的大致时长
SEARCH_SPIN_DEG = 45            # 探索中周期性扫视角度
TARGET_COLOR_REFLECTION_MAX = 40   # 目标物近处反射率上限（待标定）
TARGET_COLOR = None             # [TBD] 目标物颜色（Color.*，实机标定后填）
FINISH_COLOR = None             # [TBD] 终点色标（Color.*，实机标定后填）


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
