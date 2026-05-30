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
      task.c/h             #     任务层: 避障状态机, 对角线导航状态机
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
  2. 超声波/MPU6050 传感器更新（仅在对应任务使能时）
  3. Key_GetPressed() 处理菜单输入
  4. 根据 g_menuState 和 g_running 选择控制模式:
     - PID_control()              任务1: 纯循线
     - ObstacleAvoidance_Task()   任务2: 循线+避障
     - diagonal_Task()            任务3/4: 直线+对角线导航
  5. OLED 显示更新 (非阻塞)

中断:
  SysTick_Handler          (1ms)   → tick_ms++
  TIMA0_IRQHandler         (10ms)  → 编码器速度采样
  GROUP1_IRQHandler        (GPIOA) → 编码器正交解码
  TIMER_US_INST_IRQHandler (50us)  → 超声波回波测量
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
| 2 | 寻迹避障行驶 | 1-5圈可设 | `ObstacleAvoidance_Task()` |
| 3 | 定点直线+对角线 | 固定1圈 | `diagonal_Task()` |
| 4 | 指定路径多圈 | 固定4圈 | `diagonal_Task()` (同任务3) |

> **任务3和任务4使用同一个 `diagonal_Task()` 函数**，仅目标圈数不同（1 vs 4）。不需要两个独立的代码路径。

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
- **实现**: `ObstacleAvoidance_Task()` — 6阶段避障状态机
- 圈距: 比任务1略大（避障绕行路径更长）

### 任务3/4 — 定点直线+对角线行驶（核心难点）

路径: **A → B → D → C → A**。任务3执行 **1圈** 后停车，任务4执行 **4圈** 后停车。两者共用同一个 `diagonal_Task()` 函数，仅目标圈数 `g_targetCircles` 不同。

```
  A ————→ B        A→B: 循线直行 120cm (有引导线)
  |   ╱   |
  | ╱     |        B→D: 对角线惯性导航 ≈169.7cm (无引导线)
  |╱      |
  D ←———— C        D→C: 循线直行 120cm (有引导线)
    C→A: 对角线惯性导航 ≈169.7cm (无引导线)
```

#### 几何分析

```
A = 起点/终点
B = 右侧相邻顶点
C = 对角顶点 (A的对角)
D = 下方相邻顶点

各段航向 (假设 A→B 为 0° 正东方向):
  A→B: 航向   0° (正东),     距离 120.0 cm, 有引导线 → 循线
  B→D: 航向 135° (东南),     距离 169.7 cm, 无引导线 → 陀螺仪稳航 + 编码器计距
  D→C: 航向   0° (正东),     距离 120.0 cm, 有引导线 → 循线
  C→A: 航向 -45° (西北),     距离 169.7 cm, 无引导线 → 陀螺仪稳航 + 编码器计距

每段转弯角度:
  B点: 0° → 135° = 右转 135° (或左转 225°)
  D点: 135° → 0° = 左转 135° (或右转 225°) —— 需找线
  C点: 0° → -45° = 左转 45° (或右转 315°)
  A点: 到达后停车
```

#### 关键实现要求

1. **线段循迹 (A→B, D→C)**: 标准 PID 循线，需检测是否到达拐角（编码器累计脉冲 ≥ 120cm 对应脉冲数，或传感器全黑检测）
2. **对角线惯性导航 (B→D, C→A)**:
   - 到达拐角后先用 `TurnToAngle()` 转至目标航向
   - 沿对角线直行时用 `anglePID` 锁定航向，抵消电机差速漂移
   - 用编码器累计距离判断是否到达目标顶点（≈170cm 对应脉冲数）
   - 到达后检测边线，切换回循线模式
3. **坐标系**: MPU6050 偏航角 (yaw) 为绝对航向参考，需在起点 A 处初始化 yaw = 0

### 任务与圈距对应

三个任务走一圈的距离不同，**切换任务时自动调用 `Encoder_SetPulsesPerCircle()` 更新圈距**，确保 `Get_Current_Circles()` 返回值正确。

| 任务 | 默认每圈脉冲数 | 圈数来源 | 停车判断 |
| ---- | -------------- | -------- | -------- |
| 任务1 纯循线 | 13000 (正方形480cm，编码器×4) | 用户设定 `g_targetCircles` | `Get_Current_Circles() >= g_targetCircles` |
| 任务2 循线+避障 | 18000 (绕行路径更长) | 用户设定 `g_targetCircles` | `Get_Current_Circles() >= g_targetCircles` |
| 任务3 对角线单程 | 不适用 | 固定 1 | `diagonal_Task()` 内部状态机 → `DIAG_DONE` |
| 任务4 对角线4圈 | 不适用 | 固定 4 | `diagonal_Task()` 内部圈数计数器达4 → `DIAG_DONE` |

> **任务3/4 不依赖 `Get_Current_Circles()`**。它们在 `diagonal_Task()` 内部用分段编码器距离（120cm/170cm）+ 圈数计数器判断每段是否完成。`g_targetCircles` 在菜单选择任务3/4时自动写入 1 或 4，仅作为 `diagonal_Task()` 的目标圈数参数。

### 圈距切换机制

```c
菜单切换任务时:
  任务1 → Encoder_SetPulsesPerCircle(13000)
  任务2 → Encoder_SetPulsesPerCircle(18000)
  任务3/4 → 不使用圈距，但可设一个占位值

启动运行时:
  Encoder_ResetDistance()        // 清零累计脉冲
  diagonal_Task_Reset()          // 对角线状态机复位 (仅任务3/4)
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
| `PID_control_head(l,r)` | 纯速度环: 左右轮独立速度控制，无转向干预 |
| `TurnToAngle(targetDeg)` | 角度环: 用 anglePID 控制小车转到目标偏航角 |
| `Angle_Control(target, actual)` | 角度PID单步计算，返回 steer 值 |

## 圈距配置

圈数通过编码器脉冲累计值计算: `圈数 = 累计脉冲 / 每圈脉冲数`。

- `Encoder_SetPulsesPerCircle(uint32_t pulses)` — 切换任务时设置不同圈距
- `Encoder_ResetDistance()` — 启动运行时清零累计脉冲
- `Get_Current_Circles()` — 返回当前圈数 (float)

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
6. **双版本 Encoder**：根和 keil 的 `Hardware/Encoder.c` 均已使用 `s_totalCountA/B` 累计脉冲（不清零）来计算圈数，`Get_Encoder_countA/B` 仅用于每 10ms 的速度采样。两者各有 `Encoder_SetPulsesPerCircle()` 接口，函数签名相同。
7. **任务3/4 的惯性导航依赖 MPU6050 偏航角精度**。陀螺仪零偏标定 (`MPU6050_CalibrateGyroZ`) 对对角线精度至关重要。
