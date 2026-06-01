# CLAUDE.md — MSPM0G3507 智能小车项目

## 项目概述

基于 TI MSPM0G3507 (ARM Cortex-M0+, 80MHz) 的循线智能小车，运行在 LP_MSPM0G3507 LaunchPad 上。无 RTOS，采用超级循环 (super-loop) 架构。

**主构建为 Keil MDK** (ARMCLANG V6.21)。以下所有路径和架构描述均以 Keil 构建为准。

## 目录结构

```
car_02/
  empty.c                  # 主入口，超级循环+菜单状态机
  empty.syscfg             # SysConfig 项目文件（引脚/外设配置的唯一来源）
  ti_msp_dl_config.c/.h    # SysConfig 自动生成的初始化代码

  Hardware/                # 基础驱动模块（跨构建共享）
    Motor.c/h              #   PWM 电机驱动 (TIMG12, -100~+100)
    Sensor.c/h             #   8路红外循线传感器 (加权位置 -79~+79)
    Encoder.c/h            #   编码器速度测量 (TIMA0 10ms采样)，支持可配置圈距
    MPU6050_MSPM0.c/h      #   MPU6050 陀螺仪 (软件I2C, Z轴偏航积分)
    Ultrasonic.c/h         #   HC-SR04 超声波 (50us ISR, 非阻塞)
    OLED.c/h               #   SSD1306 OLED 128x64 (软件I2C, 帧缓冲)
    OLED_Data.c/h          #   OLED 字库和图像数据
    PID.c/h                #   通用PID算法 (PID_Init/Update/Reset)
    Key.c/h                #   4键按钮 (软件去抖)
    LED.c/h                #   板载LED
    Delay.c/h              #   SysTick 毫秒定时
    Uart.c/h               #   UART0 调试输出 (115200)

  keil/                    # [主构建] Keil MDK (ARMCLANG V6.21)
    Hardware/              #   扩展/覆盖模块 — 构建时优先链接此目录
      task.c/h             #     任务层: 避障状态机(v1), 对角线导航状态机
      task_v2.c/h         #     避障状态机 v2 (编码器+MPU6050)
      PID.c                #     完整PID: PID_control, TurnToAngle, Angle_Control 等
      Encoder.c/h          #     编码器 (与根 Hardware/ 同步维护)
      ...                  #     其余文件与根 Hardware/ 同名，部分有差异
    empty_LP_MSPM0G3507_nortos_keil.uvprojx

  gcc/                     # GCC (arm-none-eabi-gcc) 构建
  iar/                     # IAR (iccarm) 构建
  ticlang/                 # TI Clang (tiarmclang) 构建
  empty_LP_MSPM0G3507_nortos_ticlang/  # CCS/Theia 构建目录
  ultrasonic-gpio-oled-hardware-i2c/   # 独立子项目 (超声波OLED显示)
```

### 关键：双份 Hardware/ 的维护规则

- **`keil/Hardware/` 是主构建实际链接的版本**。修改驱动逻辑时必须同时更新此目录。
- **根 `Hardware/` 保留给其他构建 (GCC/IAR/TIClang)**，其中 PID.c 仅含基础 Init/Update/Reset，无控制函数。
- 当文件在两个目录中同名存在时，以 `keil/Hardware/` 为权威版本。
- 新增函数需要同时在两处声明/定义（如 `Encoder_SetPulsesPerCircle` 已在两处 Encoder.c/h 同步）。

## 架构

```
main() 超级循环:
  1. Sensor_GetQuantizedPos() → 通过8路红外计算加权线位置
  2. 超声波非阻塞测距（始终运行）
  3. g_mpu6050_flag 检查 → MPU6050_UpdateYaw() (TIMA0 10ms定时触发)
  4. Key_GetPressed() 处理菜单输入
  5. 根据 g_menuState 和 g_running 选择控制模式:
     - PID_control()              任务1: 纯循线
     - ObstacleAvoidance_Task_v2()  任务2: 循线+避障 (v2: 编码器+MPU6050)
     - CornerTurn_Task_v2()       任务3/4: 传感器检角+对角导航
  5. OLED 显示更新 (非阻塞)

中断:
  SysTick_Handler          (1ms)   → tick_ms++
  TIMA0_IRQHandler         (10ms)  → 编码器速度采样 + 置 g_mpu6050_flag=1
  GROUP1_IRQHandler        (GPIOA) → 编码器正交解码
  TIMER_US_INST_IRQHandler (50us)  → 超声波回波测量

> TIMA0 ISR 同时设置 `g_mpu6050_flag`，主循环检测到此标志时调用 `MPU6050_UpdateYaw()`。
> 这样 MPU6050 采样由硬件定时器 10ms 固定间隔驱动，不受主循环速度波动影响。
```

## 电赛校赛 — 赛题分析

### 赛道规格

- **正方形赛道**，边长 120 cm，顶点按顺时针依次为 A、B、C、D
- 赛道边线为黑色胶带，供红外循线传感器检测
- **对角线无引导线**，需靠陀螺仪+编码器实现惯性导航

```
  A(0,0) -------- B(120,0)
    |                |
    |   对角线无引导线  |
    |                |
  D(0,120) ------ C(120,120)
```

### 四项任务（三种代码路径）

| 任务 | 名称 | 圈数 | 代码路径 |
| ---- | ---- | ---- | -------- |
| 1 | 自动寻迹行驶 | 1-5圈可设 | `PID_control()` |
| 2 | 寻迹避障行驶 | 1-5圈可设 | `ObstacleAvoidance_Task_v2()` |
| 3 | 定点直线+对角线 | 固定1圈 | `CornerTurn_Task_v2()` |
| 4 | 指定路径多圈 | 固定4圈 | `CornerTurn_Task_v2()` (同任务3) |

> **任务3和任务4使用同一个 `CornerTurn_Task_v2()` 函数**，仅目标圈数不同（1 vs 4）。不需要两个独立的代码路径。

### 任务1 — 自动寻迹行驶

- 小车沿正方形边线（黑色胶带）自动循迹
- 圈数 1-5 可通过菜单设定
- 单圈行驶时间 ≤ 25 秒
- **实现**: 标准 `PID_control()`，steerPID 转向 + leftPID/rightPID 速度闭环
- 圈距(每圈脉冲数): 正方形一圈 = 4 × 120cm = 480cm，需根据车轮参数标定

### 任务2 — 寻迹避障行驶

- 在任务1基础上增加避障
- 障碍物尺寸: 15cm × 5cm × 15cm（长×宽×高），置于边线中部
- 全程不得触碰障碍物
- **避障后必须在到达拐角前回到边线**继续循迹
- 圈数 1-5 可通过菜单设定，单圈 ≤ 25 秒
- **实现**: `ObstacleAvoidance_Task_v2()` — 编码器测距 + MPU6050 转角，不依赖延时（旧版 v1 保留在 `task.c`）
- 圈距: 14300（比任务1略大，避障绕行路径更长）

### 任务3/4 — 定点直线+对角线行驶（传感器触发版）

路径: **A → B → D → C → A**。任务3执行 **1圈** 后停车，任务4执行 **4圈** 后停车。两者共用 `CornerTurn_Task_v2()`，仅 `g_targetCircles` 不同。

不再预设各段航向和距离。改为**传感器触发**：识别到直角后原地转135°越过拐角，直行到下一线段，转回135°回归循线。

```
每组操作 (过一个拐角):
  检测直角(一侧4灯全黑) → 原地转135°(朝直角方向) → 直行(至见线/超距) → 转回135°(反向)

2组 = 1圈 (从A回到A)
```

#### 直角检测

- S0~S3 全部识别到**黑线** → 左直角 → 左转 -135°
- S4~S7 全部识别到**黑线** → 右直角 → 右转 +135°
- 连续 N 帧消抖防误触

#### 直行结束条件（任一满足）

1. 编码器增量 > `CORNER_STRAIGHT_PULSES`（距离保护）
2. 中间传感器 S3 且 S4 同时识别到黑线（检测到下一条线段）

#### 关键实现要求

1. **循线段**: 标准 `PID_control()`，直到传感器检测到直角
2. **转角段**: `TurnToAngle()` 非阻塞原地转 135°
3. **直行段**: `PID_control_head()` 纯速度环直行，编码器脉冲快照计距
4. **转回段**: `TurnToAngle()` 反方向转 135°，完成后 `g_cornerSetsCompleted++`
5. **停车**: 主循环检查 `g_cornerSetsCompleted >= g_targetCircles * 2` 时停车

### 任务与圈距对应

三个任务走一圈的距离不同，**切换任务时自动调用 `Encoder_SetPulsesPerCircle()` 更新圈距**，确保 `Get_Current_Circles()` 返回值正确。

| 任务 | 默认每圈脉冲数 | 圈数来源 | 停车判断 |
| ---- | -------------- | -------- | -------- |
| 任务1 纯循线 | 13000 (正方形480cm，编码器×4) | 用户设定 `g_targetCircles` | `Get_Current_Circles() >= g_targetCircles` |
| 任务2 循线+避障 | 14300 (绕行路径更长) | 用户设定 `g_targetCircles` | `Get_Current_Circles() >= g_targetCircles` |
| 任务3 对角线单程 | 16300 | 固定 1 | `Get_Current_Circles() >= g_targetCircles` |
| 任务4 对角线4圈 | 16300 | 固定 4 | `Get_Current_Circles() >= g_targetCircles` |

> **所有任务统一用编码器圈数停车**。转角 v2 状态机用脉冲快照计距不破坏累计值，`Get_Current_Circles()` 正常增长。

### 圈距切换机制

```c
菜单切换任务时:
  任务1 → Encoder_SetPulsesPerCircle(13000)
  任务2 → Encoder_SetPulsesPerCircle(14300)
  任务3/4 → Encoder_SetPulsesPerCircle(16300)

启动运行时 (K4):
  Encoder_ResetDistance()               // 清零累计脉冲
  ObstacleAvoidance_Task_v2_Reset()     // 避障状态机复位 (仅任务2)
  CornerTurn_Task_v2_Reset()            // 转角状态机复位 (仅任务3/4)
```

各任务的每圈脉冲数需在**实车标定**后填入。标定方法：让车在赛道上跑一圈，读取编码器累计脉冲值。

## 4个PID控制器

| 控制器 | 用途 | 位置 |
| ------ | ---- | ---- |
| `steerPID` | 循线转向 (输入: linePos, 输出: 速度差) | empty.c |
| `leftPID` / `rightPID` | 左右轮速度闭环 | empty.c |
| `anglePID` | IMU航向角控制 (对角线段稳航+转弯) | empty.c |

### PID 控制函数 (keil/Hardware/PID.c)

| 函数 | 用途 |
| ---- | ---- |
| `PID_control()` | 标准循线: steerPID(转向) + leftPID/rightPID(速度闭环) |
| `PID_control_head(speed, targetYawDeg)` | 速度环+角度修正: 以目标航向驱动anglePID修正左右差速，直行中主动抗跑偏 |
| `TurnToAngle(targetDeg)` | 角度环: 用 anglePID 控制小车转到目标偏航角（非阻塞，需每帧调用） |
| `Angle_Control(target, actual)` | 角度PID单步计算，返回 steer 值 |

## 圈距配置

圈数通过编码器脉冲累计值计算: `圈数 = 累计脉冲 / 每圈脉冲数`。

- `Encoder_SetPulsesPerCircle(uint32_t pulses)` — 切换任务时设置不同圈距
- `Encoder_ResetDistance()` — 启动运行时清零累计脉冲（仅开跑时调用，避障状态机内禁止调用）
- `Encoder_GetDistancePulses()` — 返回原始累计脉冲数，用于阶段内距离增量比较
- `Get_Current_Circles()` — 返回当前圈数 (float)

## 避障 v2 状态机 (keil/Hardware/task_v2.c)

旧版 v1 (`ObstacleAvoidance_Task`) 以固定延时驱动各阶段，电压波动导致实际行走距离不一致。v2 全部改用编码器脉冲计距 + MPU6050 `TurnToAngle()` 非阻塞转角。

### 轨迹 (右避障)

障碍物在线右侧，小车从右侧绕过：

```
  右转45° → 直行绕过 → 左转90° → 直行寻线归位
```

### 6 阶段

| 阶段 | 行为 | 触发下一阶段条件 |
|------|------|-----------------|
| `AVOID2_IDLE` | 正常循线 `PID_control()` | 超声波 `dist < 25cm` |
| `AVOID2_STOP` | 停车，记录 `origin_yaw`，快照编码器 | 立即 |
| `AVOID2_TURN_RIGHT` | `TurnToAngle(origin_yaw + 45°)` | `|err| < 5°` |
| `AVOID2_FORWARD` | `PID_control_head(60,60)` 直行 | 编码器增量 > 1350 脉冲 |
| `AVOID2_TURN_LEFT` | `TurnToAngle(origin_yaw - 45°)`（实际左转 90°） | `|err| < 5°` |
| `AVOID2_SEEK_LINE` | `PID_control_head(60,60)` 寻线 | `is_lost==0` 找到线，或超时 2500 脉冲 |

### 关键设计

- **不调用 `Encoder_ResetDistance()`**：各阶段用 `g_segmentStartPulses` 快照编码器值，比较增量 `Encoder_GetDistancePulses() - g_segmentStartPulses > 阈值`。累计圈数不受影响。
- **`TurnToAngle()` 非阻塞**：每帧调用一次，状态机自行检查收敛（`angle_diff()` 归一化到 ±180°）。
- **寻线超时保护**：SEEK_LINE 阶段最多走 2500 脉冲后强制回 IDLE，防止无限寻线。

### 可调参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `AVOID_TURN_DEG` | 45 | 右转避开角度 (度) |
| `AVOID_RETURN_DEG` | 45 | 左转回线角度 (度) |
| `AVOID_DIST_PULSES` | 1350 | 绕障直行距离 (编码器脉冲) |
| `AVOID_SPEED` | 60 | 避障期间直行速度 (0~100) |
| `AVOID_OBSTACLE_CM` | 25 | 超声波障碍检测阈值 (厘米) |
| `AVOID_CONVERGE_THRESH` | 3.0f | 转向到位判定阈值 (度) |
| `AVOID_SEEK_MAX_PULSES` | 2500 | 寻线阶段最大距离，超时放弃 |

## 转向 v2 状态机 (keil/Hardware/task_v2.c)

替代空的 `diagonal_Task()`，用传感器检测直角 + 编码器计距 + MPU6050 转角实现正方形赛道对角导航。2次拐角操作 = 1圈。

### 7 阶段

```
STARTUP(地图外直行入场) → IDLE(循线) → ADVANCE(前移对齐) → TURN1(转135°) →
  STRAIGHT(直行过对角) → ADVANCE2(见线后前移) → TURN2(转回origin_yaw) → IDLE(计数+1)
```

> **STARTUP 阶段不计入圈数**。任务3/4要求在地图外A点外侧启动，K4按下后先直行400脉冲进入地图，然后 `Encoder_ResetDistance()` 清零编码器，再进入IDLE开始循迹。STARTUP 阶段的编码器增量不会影响圈数计算。

| 阶段 | 行为 | 触发下一阶段条件 |
|------|------|-----------------|
| `CORNER2_STARTUP` | `PID_control_head(60, origin_yaw)` 地图外直行入场 | 编码器增量 > 400 脉冲 → `Encoder_ResetDistance()` |
| `CORNER2_IDLE` | `PID_control()` 循线 | 一侧4灯全黑 + 3帧消抖，记录 `origin_yaw` |
| `CORNER2_ADVANCE` | `PID_control_head(60, origin_yaw)` 前移对齐旋转中心 | 编码器增量 > 350 脉冲 |
| `CORNER2_TURN1` | `TurnToAngle(origin_yaw ± 135°)` 朝对角方向转 | 误差 < 3° |
| `CORNER2_STRAIGHT` | `PID_control_head(60, origin_yaw ± 135°)` 角度修正直行 | 中双传感器(S3且S4)见线 或 编码器>4600脉冲 |
| `CORNER2_ADVANCE2` | `PID_control_head(60, origin_yaw ± 135°)` 见线后前移对齐 | 编码器增量 > 400 脉冲 |
| `CORNER2_TURN2` | `TurnToAngle(origin_yaw)` 转回原始航向 | 误差 < 3°, g_cornerSetsCompleted++, 回IDLE |

> **TURN2 直接回到 `origin_yaw`**（不是 `origin_yaw ∓ 135°`）。因为正方形对边平行，到达对边后只需回到原始航向即可沿下一段循线。

### 直角检测与转角方向

- S0~S3 全部黑线 → 左直角 → 左转 +135° (`turnDir = +1`)
- S4~S7 全部黑线 → 右直角 → 右转 -135° (`turnDir = -1`)
- 连续 3 帧消抖防误触

> 注意：MPU6050 yaw 正向为逆时针(左转)。`origin_yaw + 135` 是左转135°，`origin_yaw - 135` 是右转135°。

### 角度PID积分复位

`PID_control_head` 直行期间会持续调用 `Angle_Control`，`anglePID` 积分项逐渐累积。进入 `TurnToAngle` 前必须 `PID_Reset(&anglePID)` 清零积分，否则旧积分会抵抗新目标方向的转动。当前在 ADVANCE→TURN1、STRAIGHT→ADVANCE2、ADVANCE2→TURN2 三处转换均已加入复位。

### 停车逻辑

所有任务统一用编码器圈数: `Get_Current_Circles() >= g_targetCircles`。圈距 15700/圈，转角 v2 用脉冲快照不破坏累计值。

### 可调参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CORNER_TURN_DEG` | 135 | 转角角度 (度) |
| `CORNER_STRAIGHT_PULSES` | 4600 | 直行最大距离 (编码器脉冲，约94cm) |
| `CORNER_STRAIGHT_SPEED` | 80 | 直行速度 (0~100) |
| `CORNER_STARTUP_PULSES` | 400 | 地图外启动直行入场距离(不计入圈数) |
| `CORNER_ADVANCE_PULSES` | 300 | 检角后前移距离，对齐旋转中心 |
| `CORNER_RETURN_ADVANCE_PULSES` | 400 | 见线后前移距离，对齐后再转回 |
| `CORNER_CONVERGE_THRESH` | 3.0f | 转向到位阈值 (度) |
| `CORNER_DETECT_DEBOUNCE` | 3 | 直角检测消抖帧数 |

## 引脚分配

| 外设 | 引脚 |
|------|------|
| 电机PWM | PB20 (左), PA25 (右) |
| 电机方向(H桥) | PA24/AIN1, PA16/AIN2, PB6/BIN1, PB7/BIN2 |
| 编码器A | PA15(E1A), PA14(E1B) |
| 编码器B | PA13(E2A), PA12(E2B) |
| 循线传感器 S0-S7 | PB24, PB14, PB19, PB18, PB17, PB16, PB2, PB9 |
| OLED I2C | PA0(SDA), PA1(SCL) |
| MPU6050 I2C | PA7(SDA), PA6(SCL) |
| 超声波 | PB8(Trig), PB15(Echo) |
| 按键 K1-K4 | PA22, PA27, PA28, PA31 |
| UART0 | PA10(TX), PA11(RX) |
| LED | PA2 |

## CPU 频率: 80 MHz

## 超声波模块 — SysConfig 配置（重要）

超声波驱动使用 **TIMG0**（实例名 `TIMER_US`）产生 50us 定时中断，配合 Trig/Echo GPIO 实现非阻塞测距。

> **不要直接编辑 `.syscfg` 文件！** 必须在 SysConfig GUI 中修改后重新生成 `ti_msp_dl_config.c/.h`。

**SysConfig GUI 操作步骤：**

1. 在 SysConfig 左侧面板找到 **TIMER_US**（TIMG0 实例）
2. 修改以下参数：

| 参数 | 原值 | 目标值 |
| ---- | ---- | ------ |
| Mode | One Shot Up | **Periodic** |
| Period | 40 ms | **50 us** |
| Prescaler | 32 | **1** |
| Start Timer | 未勾选 | **勾选** |
| Interrupt Priority | — | **2** |

3. Save → 重新生成 `ti_msp_dl_config.c/.h`

**定时器占用的确认：** TIMG0 仅用于超声波，不与任何其他外设冲突。中断优先级 0=编码器GPIO、1=编码器TIMA0、2=超声波TIMG0。

## OLED 限制

- SSD1306 128×64，`OLED_8X16` 字体下最多显示 **4 行**（Y 坐标 0, 16, 32, 48）
- 每行最多 128 像素宽：中文字符约 16px 宽，ASCII 约 8px 宽
- 混排时最多约 **16 个 ASCII 或 8 个中文**（或按比例混合），超出会乱码

## 菜单系统

状态机: `MENU_MAIN → MENU_SET_LAPS / MENU_TASK_SEL → MENU_RUNNING`

| 按键 | 全局 | MAIN | SET_LAPS | TASK_SEL | RUNNING |
| ---- | ---- | ---- | -------- | -------- | ------- |
| K1 | 急停+回主菜单 | — | 回主菜单 | 回主菜单 | 急停 |
| K2 | — | 设置圈数 | 圈数+1 | 上一个任务 | 参数页+ |
| K3 | — | 选择任务 | 圈数-1 | 下一个任务 | 参数页- |
| K4 | — | 启动运行 | 确认返回 | 确认返回 | — |

K1 在任何状态下均触发急停：`g_running = 0`, `Set_PWM(0,0)`, 返回 `MENU_MAIN`。

## 编码

- 源文件编码: **UTF-8** (部分文件原为GBK，已转换)
- 注释语言: 中文
- 缩进: 4空格
- **Git 提交信息使用中文**

## 重要：修改代码时保护中文注释

**所有 `.c` / `.h` 文件均为 UTF-8 编码，包含大量中文注释。修改代码时必须遵守以下规则，避免注释损坏：**

1. **禁止使用不支持 UTF-8 的编辑器**。确保编辑器以 UTF-8 编码打开和保存文件。
2. **Keil uVision 用户特别注意**：Keil 默认编码为 ANSI/GBK，打开文件时可能不会自动识别 UTF-8。保存前务必确认编码为 UTF-8。
3. **编辑后验证**：修改任何文件后，在 git diff 中确认中文注释没有变成 `?????` 或 `锟斤拷` 等乱码。
4. **禁止使用 GBK/ANSI 编码保存**：即使只是添加一行代码，也可能导致整行中文注释损坏。始终以 UTF-8 保存。
5. **如果发现注释已损坏**（出现大量 `?` 字符）：立即用 `git show HEAD:文件路径 > 文件路径` 恢复到最新提交的干净版本，然后重新应用代码修改。

常见的编辑器设置：
- **VS Code**: 右下角状态栏确认编码为 "UTF-8"，不是 "GBK" 或 "GB2312"
- **Keil uVision**: Edit → Configuration → Editor → Encoding → UTF-8
- **Notepad++**: 编码菜单 → 转为 UTF-8 编码
- **CCS/Theia**: Window → Preferences → General → Workspace → Text file encoding → UTF-8

## 注意事项

1. **SysConfig 是引脚配置的唯一来源**。修改 `empty.syscfg` 后重新生成 `ti_msp_dl_config.c/.h`。
2. **主构建为 Keil**。`keil/Hardware/` 下的文件是实际链接版本，修改驱动时优先更新此目录。
3. **主循环无固定周期**。代码中注释掉了 `Delay_ms(10)`，实际运行频率取决于各模块耗时。
4. **OLED 和 MPU6050 共用软件 I2C 总线**，需注意访问冲突。
5. **`ultrasonic-gpio-oled-hardware-i2c/` 是独立子项目**，有自己的 main.c，不参与主构建。
6. **双版本 Encoder**：根和 keil 的 `Hardware/Encoder.c` 均已使用 `s_totalCountA/B` 累计脉冲（不清零）来计算圈数，`Get_Encoder_countA/B` 仅用于每 10ms 的速度采样。两者各有 `Encoder_SetPulsesPerCircle()` 和 `Encoder_GetDistancePulses()` 接口，函数签名相同。
7. **任务3/4 的惯性导航依赖 MPU6050 偏航角精度**。陀螺仪零偏标定 (`MPU6050_CalibrateGyroZ`) 对对角线精度至关重要。
8. **单元测试函数**：`Test_MPU6050_TurnToAngle()` 位于 `keil/Hardware/MPU6050_MSPM0.c`，声明于 `MPU6050_MSPM0.h`。用于独立测试 MPU6050 + TurnToAngle 原地转向（+90°/-90° 循环）。用法：在 `main()` 的 `while(1)` 中替换为 `while(1) { Test_MPU6050_TurnToAngle(); }`。测试结束后恢复原循环体即可。
9. **避障 v2 不得调用 `Encoder_ResetDistance()`**：各阶段用 `Encoder_GetDistancePulses()` 快照 + 差值比较的方式计距，否则会破坏累计圈数。
10. **丢线消抖**：`keil/Hardware/Sensor.c` 和 `Hardware/Sensor.c` 均已加入 5 次连续 `count==0` 消抖，防止避障回正后晃动导致误触发丢线自转。
11. **`task_v2.c` 需加入 Keil 工程**：新增源文件需手动在 Keil MDK 中添加。
