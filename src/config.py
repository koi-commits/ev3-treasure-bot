"""
config.py — 全局参数，single source of truth
Pybricks v4.1.2 / LEGO MINDSTORMS EV3

规则 (rules):
  * 所有标 [TBD] 的值都是占位符 (placeholder)，不是测量值。
  * 实物确认后：改数值 + 把对应的 *_CONFIRMED 改为 True + 记录到 docs/calibration.md
  * 其他脚本只 import 这里，不要在测试脚本里写死端口或尺寸。

2026-11-03 实机探测结果（src/diag.py，旧底盘）:
  * 电机在 Port.A 和 Port.D（B、C 空）
  * 颜色传感器在 Port.S1（S2/S3/S4 空）
  * 固件 4.1.2，电池 7477 mV

2026-11-17 硬件重建（v3）: 官方 LEGO Education EV3 "Robot Educator" 驱动底座 +
  颜色传感器朝下居中 + 陀螺仪 + V 形推板（口袋顶点装触感）。端口全部重排、**未复核**。
  换底盘会让颜色标定与巡线参数全部失效，相关行标
  "[TBD - invalidated by chassis rebuild 2026-11-17]"。
"""

from pybricks.parameters import Direction, Port

# ============================================================
# 1. Ports  —— 已由 src/diag.py 实机扫描确认
# ============================================================
# 2026-11-17 硬件 v3（官方 Robot Educator 驱动底座）：端口全部重排，尚未在实物上复核。
LEFT_MOTOR_PORT = Port.B        # [TBD] 左轮 B（旧底盘为 A）
RIGHT_MOTOR_PORT = Port.C       # [TBD] 右轮 C（旧底盘为 D）
COLOR_SENSOR_PORT = Port.S3     # 左侧朝下颜色传感器（巡线用这一个，边在左侧）
COLOR_SENSOR_RIGHT_PORT = Port.S4  # 右侧朝下颜色传感器（2026-11-24 新增；缺了 hardware.color_right = None）
TOUCH_SENSOR_PORT = Port.S1     # [TBD] 触感：V 形推板顶点（口袋深处）
GYRO_SENSOR_PORT = Port.S2      # 仅 HEADING_SOURCE = "gyro" 时使用（队里目前没有陀螺仪）
PORTS_CONFIRMED = False         # 换底盘后必须重跑 diag.py / motor_id.py 再改 True
LEFT_RIGHT_CONFIRMED = False    # 同上：左右由 motor_id.py 复核

# ============================================================
# 2. Motor direction  —— 用 test_motors.py Phase 1 确认
#    目标：给"正速度"时，该轮让小车向前滚。
#    若某个轮子转反，只改这里，不要在代码里加负号。
# ============================================================
LEFT_MOTOR_DIRECTION = Direction.CLOCKWISE    # [TBD] 沿用旧底盘确认值，换底盘后未复核
RIGHT_MOTOR_DIRECTION = Direction.CLOCKWISE   # [TBD] 同上
DIRECTIONS_CONFIRMED = False    # 重跑 motor_id.py 确认后再改 True

# 电机到轮子之间的齿轮组。None = 轮子直接装在电机输出轴上。
# 例：电机 12 齿带动轮轴 36 齿 -> [12, 36]
GEARS = None                    # [TBD] v3 官方底座看着是直连，未拆开确认

# ============================================================
# 3. Geometry (mm)  —— 先用尺子量，再用 1000mm / 360° 实测修正
#    wheel_diameter: 走不够远 -> 调小；走过头 -> 调大
#    axle_track:     转不够   -> 调大；转过头 -> 调小
#    先调 wheel_diameter，再调 axle_track。
# ============================================================
WHEEL_DIAMETER_MM = 55.5        # [TBD] Pybricks Robot Educator 示例值，未在 v3 实物上量
AXLE_TRACK_MM = 104.0           # [TBD] 同上（两轮触地点中心距）；probe_new_hw.py c 段可修正
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
SENSOR_HEIGHT_MM = None         # [TBD] 换底盘后要重新确认离地高度
SAMPLE_PERIOD_MS = 50           # 采样周期
STATS_WINDOW = 20               # 滑动窗口样本数
CAL_SAMPLES = 30                # 标定时每个表面取样数

# ↓ 2026-11-17 换底盘后传感器位置/离地高度全变，下面两个数字全部失效，只当起点保留。
BLACK_REFLECTION = 0.7          # [TBD - invalidated by chassis rebuild 2026-11-17]
WHITE_REFLECTION = 12.0         # [TBD - invalidated by chassis rebuild 2026-11-17]
COLOR_CONFIRMED = False         # 重跑 calibrate.py / probe_live.py 后再改 True
# 场地实测（src/probe_live.py，2026-11-03，手推过白边 30s）：
#   low20=0.7  high20=11.5  span=10.7  —— 黑白分离 ~16 倍，过渡在 0.5s 内完成（很陡）
# 以上全部是旧底盘的数据，v3 需重测（见 docs/calibration.md 的 v3 段）。

# ============================================================
# 5b. Touch / Gyro  —— 2026-11-17 硬件 v3 新增
# ============================================================
TOUCH_DEBOUNCE_N = 3            # 触感连续 3 个样本（3 × SAMPLE_PERIOD_MS ≈ 150 ms）才算按下
                                # [TBD] 未在实物上验证：推板顶点被物体顶住的判据还要实测
GYRO_POSITIVE_DIRECTION = None  # [TBD] +1 = gyro.angle() 增大方向与 drive.angle() 一致
                                # （drive.angle() 俯视顺时针为正）；probe_new_hw.py c 段打印建议值

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
FOLLOW_SPEED = 60               # [TBD - invalidated by chassis rebuild 2026-11-17] mm/s 沿边前进速度
APPROACH_STEER = 22             # [TBD - invalidated by chassis rebuild 2026-11-17] deg/s 朝白边侧偏转
EDGE_TURN_RATE = 130            # [TBD - invalidated by chassis rebuild 2026-11-17] deg/s 见白后原地转向
EDGE_ESCAPE_MS = 350            # [TBD - invalidated by chassis rebuild 2026-11-17] 见白后必定原地转的时长
                                # 实测：只按"转到不白为止"会在内转角处来回抖，
                                # 因为刚转出去就立刻又斜着撞回来。
EDGE_CLEAR_MS = 400             # [TBD - invalidated by chassis rebuild 2026-11-17] 逃逸后盲走时长

# mission.py 用的巡线参数组（demo_edge.py 始终用上面的已验证参数）
# "validated"：11-03 实机通过的参数。车几乎平行于边时，边侧轮子会压上白带（仿真中最深约 36 mm）
# "steep"    ：同一状态机，更大逃逸角 + 更强回转，碰边入射角更陡，轮子压白更浅（仿真最深约 29 mm），
#              慢约 20%。【只在仿真中验证】白带窄（< 40 mm）或规则不允许压线时在实机 A/B 测试。
V3_SIM_FOLLOW = (65, 18, 120, 300, 350)  # speed, approach, turn, escape, clear
FOLLOW_PROFILE = "validated"
# 巡线器（2026-11-24 两个朝下传感器）：
#   "two_sensor"（默认）：左传感器照旧驱动已验证的 EdgeFollower（边在左侧），外加安全规则：
#       巡线中**右**光斑见白/红 = 车正在越线 -> 立即朝场内（远离边）强力转向 2 × escape_ms，再继续。
#   "single"：只用左传感器（旧行为）。没有右传感器时自动退回 single。
FOLLOWER = "two_sensor"
RIGHT_GUARD_TURN_RATE = 180     # deg/s，右光斑越线时的逃逸转向率（比 EDGE_TURN_RATE 130 更强）
FOLLOW_PROFILES = {
    "validated": (22, 350, 400),     # [TBD - invalidated by chassis rebuild 2026-11-17]
    "steep": (50, 600, 300),         # [TBD - invalidated by chassis rebuild 2026-11-17]
    # 2026-11-24 离线：左光斑实测 67 mm 前 / 32 mm 左。`python tests/tune_follow_v3.py --spot 67 32`
    # （直墙模型，FOLLOW_SPEED 60 / EDGE_TURN_RATE 130）前两组不同的 (approach, escape)；
    # clear_ms 在该模型里几乎不影响结果（56 组里只有 1 组有差别），沿用 validated 的 400。
    # 【只在仿真中】完整仿真 edge_to_finish 20 seeds（光斑 67/32）：validated / side_a / side_b
    # 都有 3–4 个 seed 整车开出场地 —— 这两组**没有**解决那个问题，只是更快（67 / 54 / 48 s）。
    "side_a": (10, 200, 400),
    "side_b": (14, 200, 400),
}

# 巡线增益（比例控制，误差按 black/white 对比度归一化）
KP_NORM = 1.6                   # 归一化误差 -> 转向量比例
STEER_LIMIT = 70                # 转向量限幅 (deg/s)
EDGE_MARGIN = 6                 # 触发转向的反射率余量 (%)

# 白色边界：反射率高于阈值视为"白"（桌子外/边界线）
# 阈值由 test_color.py 标定，未标定时用 DEFAULT_EDGE_THRESHOLD
DEFAULT_EDGE_THRESHOLD = 45.0   # [TBD - invalidated by chassis rebuild 2026-11-17]
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
SENSE_MODE = "reflection"      # bring-up 默认：换底盘后 SIGNATURES 已失效，先只用反射率巡线。
                               # v3 重跑 calibrate_marker.py 拿到新签名后，再改回 "rgb"（色标模式）。

# 2026-11-10 实机标定（probe_timing + calibrate_marker + _probe_marker）：
#   probe_timing 结论 A/B/D 都随表面变化（rgb 3.6..51.3）-> 不是模式切换问题，可用单模式 rgb
#   floor  rgb_mean=(4.8,5.9,5.3) bright 2.3..8.7   refl 0.2..1.8
#   white  rgb_mean=(57.3,51.1,34.7) bright 41.3..53.0 refl 5.4..15.1（场地白线 20mm）
#   finish rgb_mean=(58.7,16.4,12.0) bright 28.3..29.3（_probe_marker 重测，50/50 识别）
# 注意：白线外侧是浅黄色木地板，rgb≈(56,47,37) 与白线几乎同色 -> 在 rgb 模式下
# 两者都算 "white"，对沿边巡线有利（越界一定看得见），但不能用颜色区分"线"与"场外"。
RGB_EDGE_BRIGHT = 25.0          # [TBD - invalidated by chassis rebuild 2026-11-17] floor/white 中点（旧底盘）
MARKER_CONFIRM = 2              # 色标须连续 2 个样本（100 ms）同类才确认。
                                # 2026-11-10 实机：原值 3 时，60 mm/s 掠过色标只有 1-2 个样本，
                                # 明明"扫到了红色"却不停车（日志里有多次孤立的 lab=finish）。
FINISH_LABEL = "finish"         # SIGNATURES 中终点色标的名字

# 红色比例兜底判据 (cr_min, cg_max, bright_min)。
# 实测覆盖同一色标的全部读数：r 48..139、g 1..18、b 2..31（rgb() 可超过 100）。
#   (59,16,12) cr=67.8 cg=18.4 bright=29.0   -> 命中
#   (129,14,6) cr=86.6 cg=9.4  bright=49.7   -> 命中
#   (48,1,2)   cr=94.1 cg=2.0  bright=17.0   -> 命中
#   white (57,51,35) cr=40.1 cg=35.8         -> 不命中
#   floor (5,6,5)    cr=31.3 cg=37.5         -> 不命中
#   暗噪声 (8,3,2)   bright=4.3              -> bright 门槛挡掉
RED_RULE = (60.0, 28.0, 8.0)    # [TBD - invalidated by chassis rebuild 2026-11-17]

# calibrate_marker.py 会打印一段可直接粘贴的 SIGNATURES。
# 特征顺序：(r, g, b, bright, cr, cg)；tol 为 None 的维度不检查。
# red 的 tol 比实测散布（±0.5）放宽到 10/8：实车行驶时光斑高度、车身俯仰都会变。
# 已核对两两不重叠：floor 的 r 上界 13.9 < red 下界 48.7；
# white 的 g 下界 41.6 > red 的 g 上界 26.4。
SIGNATURES = {   # [TBD - invalidated by chassis rebuild 2026-11-17] 旧底盘实测，v3 必须重跑 calibrate_marker.py
    "floor": {"mean": (4.8, 5.9, 5.2, 5.3, 29.7, 37.5),
              "tol": (9.1, 10.7, 10.0, 9.6, 17.3, 20.9)},
    "white": {"mean": (57.3, 51.1, 34.7, 47.7, 40.1, 35.8),
              "tol": (16.1, 9.5, 22.7, 14.4, 4.0, 9.0)},
    "finish": {"mean": (58.7, 16.4, 12.0, 29.0, 67.4, 18.8),
               "tol": (10.0, 10.0, 10.0, 8.0, 10.0, 10.0)},
}

# ============================================================
# 10. 自主任务（mission.py）—— 全部【待实机验证】
# ============================================================
# "edge_to_finish" : 寻边 -> 沿边巡线 -> 遇终点色标停车     （下周二主测）
# "survey"         : 寻边 -> 沿边巡线一段时间，只记录色标与遥测，不停车
# "calib"          : 找终点 -> 色标定位 -> 场上转角自标定（打印 turn_k），不需要物体
# "capture"        : 只做搜索 + 负载嫌疑 + tap 确认（不放物体时测误报率）
# "full"           : 找终点 -> 自标定 -> 随机反弹搜索 -> tap 确认 -> 推送 -> 交付
MISSION_MODE = "edge_to_finish"
MISSION_MAX_MS = 900000         # 整场兜底（full 模式需要数分钟）
# 启动由 mission.py 的 RIGHT 按键明确确认；pbrun.cmd 不会自动触发运动。

EDGE_SIDE = -1                  # 白边在车左侧（与已验证 demo_edge 一致）

# 寻边（随机起点/朝向）
SEEK_SPEED = 90                 # mm/s，与 demo_edge 找边速度一致
SEEK_ATTEMPTS = 6
SEEK_BACKOFF_MM = 30            # 找到边后后退量（demo_edge 的未 await 后退实际更短）
START_BACK_MAX_MM = 200         # 起步光斑就在白边上：最多倒车这么远找回地面

# 巡线监督 / 异常恢复
FOLLOW_MAX_MS = 300000          # 沿边找终点的上限。2026-11-10 实测：场地内部 1236x723，
                                # 周长约 3.9 m，而 20 mm 窄白线让 bang-bang 大部分时间在逃逸，
                                # 有效前进只有约 25 mm/s -> 一圈约 160 s。原 120 s 跑不完整圈。
                                # 300 s 给到 1.5-2 圈，电池与安全兜底由 MISSION_MAX_MS 控制。
LOST_EDGE_MS = 6000             # CRUISE 中这么久没见白 -> 认为丢边，重新直行寻边
OVERRUN_BACK_MM = 60            # 续转用完仍在白区 -> 先倒车这么多
OVERRUN_LIMIT = 4               # 连续越界恢复次数上限，超过则中止
STALL_MS = 600                  # drive.stalled() 持续这么久 -> 撞到东西，后退转开
STALL_BACK_MM = 80
STALL_TURN_DEG = 60

# 出界判定：凸（矩形）场地内，任何不见白的直线段都不可能长于对角线。
# 直行超过 ARENA_DIAG_MM 仍未见白 -> 判定已在场外，朝最近见白位置返回并穿过白带。
ARENA_DIAG_MM = 1580            # 实测场地内部 1236 x 723 mm -> 对角线 1432，乘 1.1
RETURN_IN_MM = 150              # 穿过白带回到场内后再前进这么多

# 到达终点 / 放下物体
FINISH_BACK_MM = 150            # 送达后倒车离开（V 形推板倒车 = 把物体留在原地）。
                                # 150 而不是 100：轮胎要彻底脱离推板，倒少了会被推板带走。

# 几何 —— 推送与交付要用。以下 4 个是 2026-11-17 换底盘后**实测**值（Robot Educator 底盘、
# 官方 Color Sensor Down 装在 S3、官方陀螺 S2、自定义 V 形推板 + 顶点触碰 S1）。
# 约定：从两轮轴线中点起，向前为正；横向右为正。
# 自检（tests/test_config.py 会检查，改参数时别破坏）：
#   OBJ_LEAD_MM + TIRE_RADIUS_MM <= PLOW_FRONT_MM   （物体坐在推板里，不能伸到推板外面）
#   SENSOR_OFFSET_MM < PLOW_FRONT_MM                （光斑在推板后方，先看见边再压上去）
SENSOR_OFFSET_MM = 67           # measured 2026-11-24 左光斑中心到两轮轴线中点的前向距离（旧：20 mm 居中）
SENSOR_LATERAL_MM = 32          # measured 2026-11-24 左光斑相对车体中线的横向偏移，**左为正**
                                # （mission.ahead() 与仿真都按左为正；旧注释"右为正"是错的）
SENSOR_RIGHT_OFFSET_MM = 67     # measured 2026-11-24 右光斑前向距离
SENSOR_RIGHT_LATERAL_MM = -32   # measured 2026-11-24 右光斑横向偏移（左为正 -> 右侧为负）；两光斑基线 64 mm
USE_RIGHT_SENSOR = True         # False = 只用左传感器（对照 / 右传感器坏了）
OBJ_LEAD_MM = 100               # [TBD] 物体坐进推板后，物体中心到两轮轴线中点的前向距离（M8）
                                # 待复核：必须满足 OBJ_LEAD_MM + 33.5 <= PLOW_FRONT_MM
PLOW_FRONT_MM = 143             # measured 2026-11-17 推板最前端到两轮轴线中点的前向距离（M11）
                                # => 前横梁 CB = 143 - 116 = 27 mm（见 tests/sim.py GEOM）
PLOW_MOUTH_MM = 105             # [TBD - design value for the 143 mm plow (walls at +-56 mm centre), measure on robot]
# 由上面推出（不要单独改）：侧墙中心线的横向半宽 = 通口一半 + 半根梁（3.5 mm）= 56 mm。
# 推板围栏（geofence）用它当推板前角，仿真 GEOM / 覆盖率统计也用它。
PLOW_TIP_HALF_MM = PLOW_MOUTH_MM / 2.0 + 3.5
TIRE_RADIUS_MM = 33.5           # measured 2026-11-17 目标物 = 平放 LEGO 轮胎 Ø67 x 33
TIRE_DIAMETER_MM = 67.0
SENSOR_HEIGHT_MM = 8.0          # measured 2026-11-17 颜色传感器镜面离地高度（只记录，不参与计算）

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
MARKER_SIDE_MM = 150.0          # [TBD - measure the paper] 方形红纸边长；纸压在白色边界线上
MARKER_SCAN_STEP_MM = 100.0     # 找色标时垂直撞边的沿墙步距（必须 <= 纸边长才能保证撞到）
MARKER_PROBE_STEP_MM = 25.0     # 找到后沿墙小步探测纸的沿墙范围（再二分细化）
APPROACH_STANDOFF_MM = 250      # 交付前先到目标点正前方（场内侧）这么远的待命点，再直线推进
# 场上转角自标定（full 模式，记录终点后在同一条边上做一次，约 20 s）
SELF_CAL = False                # v3 默认不走整段自标定（含原地 720° 旋转）；要复现旧路径把
                                # USE_SELF_CAL 也打开（见第 11 节），两处都开才会执行。
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
# 11) Mission v3（陀螺航向 + 车道扫描 + 触碰捕获）—— 2026-11-17 换底盘后
# ------------------------------------------------------------
# 旧的四条路径（负载嫌疑 + tap/seat 确认、learned_capture/GRU 影子、对角线反弹搜索、
# 720° 原地自标定）文件都留着，但默认路径上不再走：下面四个开关默认全 False。
USE_LOAD_CAPTURE = False        # 旧：LoadMonitor + tap_confirm 座位确认
USE_LEARNED_CONTACT = False     # 旧：learned_capture / contact_inference / GRU 影子
USE_BOUNCE_SEARCH = False       # 旧：bounce() 对角线随机反弹搜索
USE_SELF_CAL = False            # 旧：calibrate_and_map() 原地 720° 自标定

# ---- 航向 ----
# 航向来源（sweep / capture / full）："odometry" = drive.angle()（轮子编码器，需要
# AXLE_TRACK_MM 误差 < 1%，见 calibrate_geometry.py）；"gyro" = gyro.angle() × GYRO_POSITIVE_DIRECTION。
# 2026-11-24：队里没有陀螺仪 -> 默认 odometry，任何地方都不读陀螺。hardware.gyro 为 None 时也走 odometry。
HEADING_SOURCE = "odometry"     # "odometry" / "gyro"
GYRO_SETTLE_MS = 2000           # 仅 gyro：起步先静置这么久再 reset 陀螺（EV3 陀螺上电有漂移）
# 边界航向修正（odometry 漂移）：同一条已知墙上的两次**白线**撞线点（红纸内沿不在白线上，
# 不参与），在里程计坐标里连线应当平行于
# 场地轴向；偏差就是两次撞线之间累计的航向误差。只在墙估计可信时用（墙已观测、两点都落在
# 墙估计 HEADING_FIX_WALL_TOL_MM 以内、轴向不是从航向猜的），单次最多 HEADING_FIX_MAX_DEG。
HEADING_FIX = False             # 默认关：20 seeds 离线没有测到收益（见 PR #7），实机有需要再开
# 两光斑对线（line squaring）：车道朝墙直行，左右光斑先后压线的里程差 -> 车头相对墙法向的角度
# atan2(d_left - d_right, 基线)。用来修正里程计航向（不需要陀螺）。只在空推板、轴向是拟合出来的、
# 两个光斑都是白线（红纸内沿不在白线上）时用；单次最多 SQUARE_MAX_FIX_DEG。
LINE_SQUARING = True
SQUARE_BACK_MM = 20.0           # 第一次见线后先退这么多，再慢速前进测两个光斑（时间分辨率 -> 1 mm）
SQUARE_SPEED = 20.0             # mm/s：50 ms 周期 = 1 mm 一次采样（≈ ±1.3° 角度分辨率）
SQUARE_CREEP_MAX_MM = 60.0      # 慢速前进上限（基线 64 mm × tan 15° ≈ 17 mm + 退回的 20 mm + 余量）
SQUARE_MAX_FIX_DEG = 15.0
SQUARE_MIN_FIX_DEG = 0.5        # 小于它当噪声，不改航向
HEADING_FIX_MAX_DEG = 10.0
HEADING_FIX_MIN_DEG = 1.0       # 死区：撞线点沿法向只能定位到一个周期的行程（100 mm/s × 50 ms
                                # = 5 mm），300 mm 跨度上约 ±1°。小于它的偏差当噪声，不修正。
HEADING_FIX_MIN_SPAN_MM = 300.0 # 两次撞线沿墙至少隔这么远（太近时角度被位置噪声放大）
HEADING_FIX_WALL_TOL_MM = 60.0
HEADING_FIX_LOOKBACK = 8        # 只和最近几次撞线配对（中间转向越多，误差越不是常数）
HEADING_FIX_PHASES = ("lane", "edge_walk", "marker_scan", "reacquire_sweep")
KP_HEADING = 2.0                # hold_heading 的比例增益（deg/s per deg）
MAX_TURN_RATE = 120             # hold_heading 的转向率限幅（deg/s）

# ---- 捕获：触碰传感器 ----
USE_TOUCH = True                # 捕获 = 触碰挡板被压住（不再用负载嫌疑 + tap 确认）
LOST_MS = 400                   # 推送中连续这么久没压住 = 物体丢了

# ---- 定场地轴向：在边上采样白带点拟合（不贴边巡线） ----
# 理由：光斑只前伸 20 mm，而轮子横向在 ±57 mm，贴边巡线时内侧轮心必然越过 40 mm 白带。
EDGE_PROBE_N = 4                # 采样几个白带点（三点定线，多采一个当冗余）
EDGE_PROBE_MM = 900.0           # 每次采样直行的上限（防止在场上空跑太远）
EDGE_PROBE_BACKOFF_MM = 120.0   # 每次采样后的倒车量（要够大，采样点基线才有 100 mm 以上）
EDGE_PROBE_TURN_DEG = 40.0      # 第二次采样前的转向（两个白带点的横向间隔 = BACKOFF * tan）
EDGE_PROBE_SHIFT_MM = 260.0     # 第三次采样前沿边挪的距离（基线越长，拟合方向越准）

# ---- 找色标：沿场地内侧的"垂直撞边"巡逻 ----
# 不用贴边巡线：光斑只比轴心前伸 20 mm，而轮子横向在 ±57 mm，车头平行白带时内侧轮
# 会压进 40 mm 宽的白带（仿真实测 51-57 mm ⇒ 轮子出界）。垂直撞边时两个轮子和光斑
# 的离边深度一致，轮子最多跟光斑一样深，永远在白带内沿以里。
EDGE_WALK_MM = 200.0            # 每次沿边走多远（要小于色标长度，否则两撞之间会跳过色标）
EDGE_WALK_MS = 300000.0         # 一圈走完的上限（整圈周长约 3.9 m，按 100 mm/s 算 40 s）
MARKER_SCAN_BACKOFF_MM = 40.0   # 找色标时退到离墙约 60 mm（色标带宽 60 mm，光斑要压上去）

# ---- 车道扫描（boustrophedon，从边上出发、车道垂直于这条边） ----
LANE_SPACING_MM = 70            # [TBD] 相邻车道间距；必须小于推板通口 PLOW_MOUTH_MM
LANE_MAX_MM = 1236              # 单条车道的上限=场地内部长边（车道必然在这么远内撞到对面白带）
LANE_MAX_COUNT = 40             # 车道数上限
LANE_CLEAR_MM = 220.0           # 开第一条车道前，沿车道方向先开这么远离开白带 [TBD]
LANE_END_MM = 150.0             # 车道一出发不到这么远就见白 = 这条墙到头了（转角的白带）
SWEEP_PREVIEW_LANES = 4         # MISSION_MODE="sweep" 只扫几条车道（3–5，用来观察色标估计）
BACKOFF_MM = 90                 # 到边/到围栏后先倒车这么多，再转向
DELIVER_TOL_MM = 8.0            # 交付收尾：物体中心距目标点小于这么多就停车（±10 mm 判定窗）
SETTLE_FOLLOW_MS = 8000.0       # 旧做法：定轴向之前先沿边走这么久（v3 已改 edge_probe，留作对照）
MARKER_FIND_MS = 120000.0       # 旧做法：full 先沿边走一圈找色标（v3 改由扫描覆盖，留作对照）

# ---- 色标（红点）估计 ----
MARKER_HIT_DEDUPE_MM = 15       # 相邻两次红点记录的最小间距
MARKER_MIN_HITS = 3             # 少于这么多个红点就不认为"知道色标位置"
MARKER_REJECT_MM = 150          # 距中位数超过这么远的红点当离群点丢掉

# ---- 场地先验（[TBD] 实测：内部 1236 x 723 mm） ----
ARENA_INTERIOR_W_MM = 1236
ARENA_INTERIOR_H_MM = 723


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

# Stop the plow front before the estimated white inner boundary.
GEOFENCE_MARGIN_MM = 20
# Parked-tyre recovery starts by reversing until the plow fence has this much
# room (the marker probes end at the wall with the fence off); capped.
RECOVER_FENCE_ROOM_MM = 30.0
RECOVER_BACKOUT_MAX_MM = 200.0
