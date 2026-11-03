"""
config.py — 全局参数，single source of truth
Pybricks v4.1.x / LEGO MINDSTORMS EV3

规则 (rules):
  * 所有标 [TBD] 的值都是占位符 (placeholder)，不是测量值。
  * 实物确认后：改数值 + 把对应的 *_CONFIRMED 改为 True + 记录到 docs/calibration.md
  * 其他脚本只 import 这里，不要在测试脚本里写死端口或尺寸。
"""

from pybricks.parameters import Direction, Port

# ============================================================
# 1. Ports  —— 用 check_env.py 扫描确认
# ============================================================
LEFT_MOTOR_PORT = Port.B        # [TBD] 左轮电机
RIGHT_MOTOR_PORT = Port.C       # [TBD] 右轮电机
COLOR_SENSOR_PORT = Port.S3     # [TBD] 朝下的颜色传感器
PORTS_CONFIRMED = False

# ============================================================
# 2. Motor direction  —— 用 test_motors.py Phase 1 确认
#    目标：给"正速度"时，该轮让小车向前滚。
#    若某个轮子转反，只改这里，不要在代码里加负号。
# ============================================================
LEFT_MOTOR_DIRECTION = Direction.CLOCKWISE    # [TBD]
RIGHT_MOTOR_DIRECTION = Direction.CLOCKWISE   # [TBD]
DIRECTIONS_CONFIRMED = False

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

BLACK_REFLECTION = None         # [TBD] test_color.py 标定后填写 (%)
WHITE_REFLECTION = None         # [TBD]
COLOR_CONFIRMED = False

# ============================================================
# 6. Safety
# ============================================================
LOW_BATTERY_MV = 7000           # 低于此电压给出警告；低电量会让速度/标定漂移


def black_white_threshold():
    """黑白阈值 = 两者中点。未标定时返回 None，调用方必须处理。"""
    if BLACK_REFLECTION is None or WHITE_REFLECTION is None:
        return None
    return (BLACK_REFLECTION + WHITE_REFLECTION) / 2


def unconfirmed_items():
    """返回还没在实物上确认的参数组名称，测试脚本启动时会打印出来。"""
    items = []
    if not PORTS_CONFIRMED:
        items.append("ports")
    if not DIRECTIONS_CONFIRMED:
        items.append("directions")
    if not GEOMETRY_CONFIRMED:
        items.append("geometry")
    if not COLOR_CONFIRMED:
        items.append("color")
    return items
