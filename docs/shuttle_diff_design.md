# OmniX 全向测量船 — 飞控运行逻辑与摆渡（Ferry）功能对接说明

> 本文档面向 **QGC 地面站开发团队**，完整描述飞控端（ArduPilot Rover 4.8 定制固件）
> 的船体驱动逻辑、三种运行模式，以及**摆渡（Ferry）**功能的飞控侧实现。
> 目标是让 QGC 团队清楚：**飞控能做什么、摆渡怎么对接、边界条件是什么**。
>
> 版本：2026-10-24 · 对应固件分支 `main`（差速船三模式 + QGC 摆渡对接）

---

## 一、船体硬件与电机布局

### 1.1 船型

- **OmniX 全向测量船**（X 型四推进器布局）
- 船体中央安装 **ADCP**（声学多普勒流速剖面仪），用于水流测量
- 需求约束：**测量时船中水流必须尽量干净** → 非悬停状态下尽量只激活远离船中的电机对
- 飞控机架参数：`FRAME_CLASS=2`（Rover/Boat）、`FRAME_TYPE=2`（OmniX）

### 1.2 电机接线（真实物理，已实测确认）

| 电机编号 | 伺服通道 | 物理位置 | OmniX 混合系数 (油门/转向/横向) |
|---|---|---|---|
| M0 | SERVO1 (CH33) | **右前** | +1 / −1 / −1 |
| M1 | SERVO2 (CH34) | **右后** | +1 / +1 / −1 |
| M2 | SERVO3 (CH35) | **左后** | +1 / +1 / +1 |
| M3 | SERVO4 (CH36) | **左前** | +1 / −1 / +1 |

- 四个推进器朝 X 四角外侧喷水
- 系数约定：正油门=前进、正转向=右转（顺时针）、正横向=向右
- **后电机对 = M1/M2（右后 + 左后）**，**前电机对 = M0/M3（右前 + 左前）**

### 1.3 航向源

- **真机**：UM982 双天线 GPS 提供航向（`GPS1_TYPE=25`，双天线定向）
- **SITL 仿真**：用默认罗盘模拟航向（真机双天线基线在仿真中不模拟）
- 因此 SITL 参数集里 `COMPASS_ENABLE=1`、`EK3_SRC1_YAW=1`

---

## 二、飞控驱动架构（怎么把"转向+油门"变成四路 PWM）

### 2.1 控制链路（ArduPilot Rover 4.8 原生）

```
期望位置/航点
   → AR_WPNav（S-Curve 路径规划，输出期望速度 + 期望转向率）
   → AR_PosControl（位置 P + 速度 PID，输出期望速度 + 期望转向率）
   → AR_AttitudeControl（转向率 PID → steering ∈ [-1,1]；速度 PID → throttle ∈ [-1,1]）
   → AP_MotorsUGV::output_omni()（按"电机对"混控 → 4 路 PWM）
   → 船体
```

### 2.2 电机对（MotorPair）机制（本固件定制）

`AP_MotorsUGV` 新增电机对概念，`output_omni()` 按当前电机对决定哪些电机参与：

| 电机对 | 激活电机 | 未激活电机 | 用途 |
|---|---|---|---|
| `ALL` | M0+M1+M2+M3 | 无 | 全向悬停 / 全向平移 |
| `REAR` | M1 + M2（后对） | M0/M3 输出 TRIM(1500) 停转 | 手动 / AUTO / 摆渡去程 差速航行 |
| `FRONT` | M0 + M3（前对） | M1/M2 输出 TRIM(1500) 停转 | 摆渡回程 倒车差速航行 |

- 差速公式（与原生 skid steering 完全一致）：

```
scaled_throttle = throttle * 0.01          # -1 ~ +1
scaled_steering = steering / 4500.0        # -1 ~ +1
左电机 = scaled_throttle + scaled_steering
右电机 = scaled_throttle - scaled_steering
REAR : 左=M2, 右=M1
FRONT: 左=M3, 右=M0
```

- 正 steering = 左侧电机更快 = **右转**，与前后位置无关（倒车时符号天然一致）

---

## 三、三种运行模式（核心逻辑）

### 3.1 手动模式（MANUAL）

- 进入：`set_motor_pair(REAR)` → 只用后两电机 M1/M2 差速
- 遥控：RC1=转向、RC3=油门，与原生差速船完全一致
- **禁用横移**：`set_lateral(0)`（差速船无横移能力）
- 退出：恢复 `ALL`

### 3.2 自动模式（AUTO）

- 进入：先 `ALL`（船型 is_boat → 先进入 loiter 全向悬停等待任务）
- **WP 子模式**：切 `REAR`（后电机差速），走原生 `navigate_to_waypoint()`
  （AR_WPNav S-Curve → `calc_steering_from_turn_rate` → 差速）
- **到点悬停**：`wp_nav.reached_destination()` 后切回 `ALL` 并进入 loiter（四电机全向定点）
- Loiter 子模式：`ALL`（全向悬停）

### 3.3 悬停（LOITER / HOLD）

- 进入：`set_motor_pair(ALL)` → 四电机全向
- 全向位置保持：按 `omni_translate_enabled()` 走 `calc_translate_to_bearing()`
  （船头锁定 + 横向平移纠偏），或原生航向转向逻辑
- 这是 **唯一四电机全部参与** 的场景

---

## 四、摆渡（Ferry）功能 — 飞控侧完整逻辑

### 4.1 对接方式（重要结论）

> **QGC 摆渡完全走 ArduPilot 标准 GUIDED 导航，飞控不需要任何新增 MAVLink 协议。**
> QGC 点"启动"发 `MAV_CMD_DO_REPOSITION`（或回退：切 GUIDED + 写单点 MISSION_ITEM）。
> 到达判定在 **QGC 本地**（距离 < 5m 且 船速 < 0.3m/s）。

### 4.2 飞控收到 DO_REPOSITION 后的完整流程

```
QGC 发 MAV_CMD_DO_REPOSITION（含切 GUIDED 标志）
  ↓ GCS_MAVLINK_Rover::handle_command_int_do_reposition
  ↓（已移除旧 AP_Shuttle 校验钩子，完全走原生）
  ↓
切到 GUIDED 模式（若不在）
  ↓
ModeGuided::set_desired_location(target)
  ├─ 计算目标相对船头方位 yaw_error = wrap_180(target_bearing - 当前航向)
  ├─ |yaw_error| ≤ 90°（目标在前方）
  │    → set_motor_pair(REAR)  后电机差速前进
  │    → set_reversed(false)   正常前进
  ├─ |yaw_error| > 90°（目标在后方）
  │    → set_motor_pair(FRONT) 前电机差速
  │    → set_reversed(true)    ★原生倒车导航：不掉头，倒着走
  └─ 走原生 AR_WPNav S-Curve 导航链路
  ↓
航行中：ModeGuided::update() → navigate_to_waypoint()
  ↓
到达（wp_nav.reached_destination()）
  → start_loiter() → ModeLoiter::_enter() → set_motor_pair(ALL) 四电机全向悬停
  → QGC 本地判定"已停靠"
```

### 4.3 摆渡 A↔B 往返的电机对切换

| 航程 | 目标相对船头 | 电机对 | 驱动方式 | 是否掉头 |
|---|---|---|---|---|
| A → B（去程） | 在船头前方 | REAR（M1/M2） | 后电机差速前进 | 否 |
| B → A（回程） | 在船尾后方 | FRONT（M0/M3） | 前电机差速**倒车** | **否**（直线返回） |

- **关键机制**：回程用 `AR_WPNav::set_reversed(true)` 原生倒车导航
  - `get_speed() < 0` → 油门为负 → FRONT 电机反转，把船往船尾方向拉
  - 位置控制器已自动处理倒车时的转向符号（`turn_rate = lat_accel / 负速度`）
  - 结果：**船头始终朝向原方向，船体直线后移回 A，全程不掉头**
- **电机对在设目标时判定一次并锁定**，整条航程不切换，避免航行中抖动

### 4.4 到达悬停

- GUIDED 到达后进入 loiter，`ModeLoiter::_enter()` 强制 `ALL` 四电机全向
- 四电机全向位置保持 → 船稳定停在目标点
- QGC 本地判定（<5m 且 <0.3m/s）显示"已停靠 X"

### 4.5 边界条件与安全

- **中途手动接管**：切 MANUAL/HOLD/其他人工模式即退出 GUIDED，航行中断，安全
- **无位置估计**：GUIDED 无法进入（原生 requires_position），故障保护（HOLD/RTL）兜底
- **DO_REPOSITION 参数**：`param1`=速度（可选，尽力而为）、`param2`=切模式标志、`x/y`=目标经纬度
- **速度**：若 `param1` 为正则设置航程速度，否则用当前 `WP_SPEED`

---

## 五、与旧"两点穿梭"的关系（兼容说明）

| 项目 | 旧两点穿梭（已废弃协议） | 新摆渡（Ferry） |
|---|---|---|
| 触发协议 | `MAV_CMD_USER_1` 子命令（设 A/B、切换、采样） | **标准 `MAV_CMD_DO_REPOSITION`** |
| 状态机 | 飞控侧 AP_Shuttle 状态机 | **无**（飞控只响应 GUIDED 目标） |
| 到达判定 | 飞控侧距离+速度 | **QGC 本地**（<5m 且 <0.3m/s） |
| A/B 点来源 | 飞控记忆 A/B | **QGC 任务航点**（A=航点1、B=航点2、测量点=航点3…） |
| 测量点 | 飞控模拟任务 | 真实航点（QGC 插入航线） |

- 飞控端 `MAV_CMD_USER_1` 处理代码**保留但仅供旧版 QGC 兼容**，新版 QGC 不发送
- `DO_REPOSITION` 已移除旧 AP_Shuttle 的距离校验/状态机钩子，**不会被拒绝**

---

## 六、SITL 仿真支持

`libraries/SITL/SIM_Rover.cpp` 的 OmniX 模型支持两种差速物理：

| 模式 | 检测条件 | 物理模型 |
|---|---|---|
| REAR | M0/M3 停转，M1/M2 有输出 | 差速船（skid）模型，与原生一致 |
| FRONT | M1/M2 停转，M0/M3 有输出 | 差速船（skid）模型，对称处理倒车 |
| ALL | 四电机都有输出 | X 形全向逆混控（横移+转向+油门） |

- 仿真物理与控制代码同源，SITL 行为 = 真机预期行为
- 启动方式：`sim_vehicle.py -v Rover -f rover-omnix` 或直接运行 SITL 二进制

### 6.1 差速船低速转向物理（重要修复）

**差速分支必须用 skid 比例偏航模型，不能用 `calc_yaw_rate()`。**

- `rover-omnix` 模型 `skid_steering = false`
- `calc_yaw_rate(steering, speed)` 在 `speed≈0` 时返回 0 → 差速船低速无法原地转向
- 症状：电机在转（REAR 差速输出正常）但船不动、Yaw 恒定 → 导航器转向饱和死锁
- 修复：`yaw_rate = constrain_float(steering * skid_turn_rate, -MAX_YAW_RATE, MAX_YAW_RATE)`
  （与原生 skid steering 一致，差速驱动的核心能力：静止也能原地转向）
- 验证：AUTO 航点导航正常，速度 0→1.28 m/s，航向可修正，到达航点 ✓

---

## 七、关键参数表（SITL 参数集 + 真机通用）

| 参数 | 值 | 说明 |
|---|---|---|
| `FRAME_CLASS` / `FRAME_TYPE` | 2 / 2 | OmniX 船型 |
| `OMNI_LAT_PCT` | 100 | 全向横向推力百分比 |
| `OMNI_YAW_HOLD` | 0 | 悬停时是否锁定船头（0=可漂移） |
| `OMNI_VEL_GAIN` | 0.25 | 全向速度环加速度反馈增益 |
| `WP_SPEED` | 1.5 m/s | 航点巡航速度 |
| `WP_RADIUS` | 1 m | 到达判定半径 |
| `WP_ACCEL` / `WP_JERK` | 0.8 / 1.0 | S-Curve 加减速/加加速度 |
| `ATC_STR_RAT_P/I/FF` | 0.5 / 0.2 / 0.2 | 转向率 PID |
| `PSC_VEL_P` / `PSC_POS_P` | 1.0 / 0.1 | 位置环/速度环增益 |
| `ATC_SPEED_P` | 0.5 | 速度 PID |
| `CRUISE_SPEED` / `CRUISE_THROTTLE` | 1.5 / 50 | 油门标定 |

---

## 八、给 QGC 团队的对接验证清单

| # | 项 | 预期 |
|---|----|------|
| 1 | QGC 摆渡 A→B | REAR 后电机差速前进，直线到达不掉头 |
| 2 | QGC 摆渡 B→A | FRONT 前电机倒车，直线返回不掉头 |
| 3 | 到达悬停 | 四电机全向，稳在目标点，QGC 显示"已停靠" |
| 4 | 中途手动接管 | 切 MANUAL/HOLD 立即退出 GUIDED |
| 5 | `DO_REPOSITION` 返回 | `MAV_RESULT_ACCEPTED`，不被任何参数拒绝 |
| 6 | 测量点插入后摆渡 | 测量点=真实航点，GUIDED 正常到达 |

---

## 附：飞控侧代码改动文件索引

| 文件 | 改动 |
|---|---|
| `libraries/AR_Motors/AP_MotorsUGV.h/.cpp` | MotorPair 枚举 + output_omni 差速分支 |
| `Rover/mode_manual.cpp` | 手动 REAR 差速 + 禁横移 |
| `Rover/mode_auto.cpp` | AUTO-WP REAR、到点 ALL 悬停 |
| `Rover/mode_guided.cpp` | **摆渡核心**：set_desired_location 按目标方位自动选 REAR/FRONT + reversed |
| `Rover/mode_loiter.cpp` | 进入悬停强制 ALL 全向 |
| `Rover/GCS_MAVLink_Rover.cpp` | DO_REPOSITION 移除旧穿梭钩子；USER_1 保留兼容 |
| `libraries/SITL/SIM_Rover.cpp` | REAR/FRONT 差速物理 + ALL 全向物理 |
