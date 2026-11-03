# EV3 Treasure Bot — 自主寻宝与定点运输

LEGO MINDSTORMS EV3 双电机差速小车 + 1 个朝地颜色传感器，Pybricks v4.1.x MicroPython。

**当前阶段：Phase 1 — 基础硬件验证**（不含搜索/寻宝/返航算法）

## Directory

```
ev3-treasure-bot/
├── README.md
├── docs/
│   └── calibration.md     # 实物参数记录表（手填）
└── src/                   # 全部在 EV3 上运行
    ├── config.py          # 端口/方向/轮径/轮距/速度/阈值，唯一参数来源
    ├── common.py          # log、按键等待、电池检查
    ├── check_env.py       # Step 0：固件信息 + 端口扫描（只读）
    ├── test_motors.py     # Step 1：电机方向与基础运动
    └── test_color.py      # Step 2：黑白反射光标定与实时读数
```

每个 `test_*.py` 都可独立运行，只依赖 `config.py` 和 `common.py`。Pybricks Code 运行时会自动把被 import 的同目录文件一起下载到 EV3，所以三个文件放在同一工作区即可。

## Environment（Pybricks v4.x on EV3 的关键区别）

| | 旧版 EV3 MicroPython (v2.0) | 本项目 (v4.1.x) |
|---|---|---|
| 运行方式 | microSD 卡上的 ev3dev Linux | 直接刷入 EV3 的 Pybricks 固件 |
| 编辑器 | VS Code + EV3 MicroPython 扩展 | 浏览器 Pybricks Code（Chrome/Edge） |
| 首行 | `#!/usr/bin/env pybricks-micropython` | **不需要** |
| 字体/图像 | `pybricks.media.ev3dev` | **不存在**，不要 import |
| 异步 | 无 | `async/await` + `run_task()` |

**不要混用：** 网上很多 EV3 教程是 v2.0（ev3dev）写法，看到 shebang、`pybricks.media.ev3dev`、`ev3brick as brick` 就是旧版。

### Step A — 确认 EV3 上现在是什么系统（只看，不改）

1. 开机看屏幕：
   - 顶部显示 Pybricks 界面/hub 名称 → 已是 Pybricks 固件，**直接跳到 Step B**
   - LEGO 原厂菜单 → 原厂固件
   - 插着 microSD 且出现 ev3dev 启动画面 → 旧版 v2.0 方案
2. 若不是 Pybricks v4.x：**先问老师/实验室管理员**，确认允许后再由有权限的人刷机。本项目代码不会、也不应该自动刷固件。（刷机走 Pybricks Code 网页的 *Install Pybricks Firmware* 向导，USB 连接；需要恢复原厂固件时也有对应选项。）

### Step B — 连接与运行

1. Chrome / Edge 打开 Pybricks Code（stable 版 `code.pybricks.com`；若学校设备用的是 beta 固件，则用 `beta.pybricks.com`，**网页版本要与固件版本匹配**）。
2. 用 mini-USB 线连接 EV3 → 点工具栏的连接按钮 → 选择 EV3。
   - Linux 需要 udev 规则；Windows/macOS 一般即插即用。
   - 4.1 起也支持蓝牙，阶段一建议先用 USB，最稳定。
3. 在 Pybricks Code 中新建 5 个文件，名称与 `src/` 完全一致，粘贴内容。
4. 打开要运行的脚本 → 点 **Run**。终端输出在网页底部，同时显示在 EV3 屏幕。
5. **CENTER 键 = 紧急停止**；RIGHT 键 = 下一步。

### Step C — 记录版本

运行 `check_env.py`，把屏幕/终端里的固件版本、hub 名称抄到 `docs/calibration.md`。以后所有问题排查都以这个版本为准。

## Run order & acceptance

| 步骤 | 脚本 | 操作 | 通过标准 |
|---|---|---|---|
| 0 | `check_env.py` | 直接运行 | 列出 2 个电机口 + 1 个颜色传感器口；电池 ≥ 7000 mV；改好 config 第 1 节 |
| 1 | `test_motors.py` P1 | **架空小车** | 左右轮各转 1 圈，都让车"向前滚"；`angle` 读数 ≈ 360 |
| 2 | `test_motors.py` P2 | 放地面 | 前进 200 mm 后退回到起点（目测误差 < 2 cm）；先右转后左转方向正确，回到原朝向（< 10°） |
| 3 | `test_motors.py` P3 | 放地面 | 低速直行 2 s 平稳无抖动，`angle` 读数接近 0 |
| 4 | `test_color.py` | 黑→白标定 | 终端显示 `PASS`：分离度 ≥ 20，静止 std ≤ 3；Live 模式手推跨边界时 BLACK/WHITE 切换干净，无反复跳变 |
| 5 | 重复 | 每个测试连跑 3 次 | 结果一致 → 阶段一完成 |

Phase 1 fail 时 → 只改 `config.py` 里的 `*_DIRECTION`，不要在代码里加负号。

## Calibration rules

- `config.py` 中所有 `[TBD]` 均为占位值，**未经测量**。
- 实测后：改数值 → 把对应 `*_CONFIRMED` 设为 `True` → 抄进 `docs/calibration.md`。
- 轮径/轮距修正（官方方法）：`straight(1000)` 走不够 → 轮径调小，走过头 → 调大；`turn(360)` 转不够 → 轮距调大，转过头 → 调小。先调轮径，再调轮距。打开 `test_motors.py` 的 `RUN_CALIBRATION = True` 运行 Phase 4。

## Notes on async

- 带 `await` 标记的 API（`run_angle`、`straight`、`turn`、`wait`、`reflection`、`color`、`hsv`、`speaker.beep` …）在 `async def` 里必须写 `await`。
- 非 awaitable（`run`、`stop`、`drive`、`angle`、`distance`、`buttons.pressed`、`screen.print` …）**不要**加 `await`。
- 程序入口统一：`run_task(main())`，不嵌套调用 `run_task`。

## Docs

- API：https://docs.pybricks.com/en/stable/ （v4.1.1）
- EV3 设备：`ev3devices` / `hubs.EV3Brick` / `robotics.DriveBase` / `tools`
