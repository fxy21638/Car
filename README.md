# MSPM0G3507 智能小车项目技术文档

## 1. 项目概述

本项目是基于 `TI MSPM0G3507` 的双电机小车控制工程，当前代码包含以下功能模块：

- 双路电机 PWM 驱动与方向控制
- 双路编码器测速
- 8 路循迹传感器读取
- 0.96 寸 OLED 显示
- 4 路按键输入
- HC-SR04 类超声波测距
- MPU6050 陀螺仪 Z 轴角速度积分得到航向角
- 串口调试输出
- 速度 PID、转向 PID、角度控制与避障任务代码

当前工程已适配 `Keil MDK + ArmClang`，目标器件为 `MSPM0G3507`。

## 2. 当前代码状态

这部分很重要，决定你上电后看到的行为。

- `empty.c` 当前 `main()` 运行的是一个 `PID` 演示循环，不是完整的小车自动循迹主循环。
- 演示循环里使用 `fake_speed` 模拟编码器速度，并在 OLED 上显示 PID 输出。
- `H24`、`PID`、避障、转向、超声和 MPU6050 的相关代码已经存在，当前主循环通过“一行任务入口”方式选择运行任务。

也就是说：

- 现在工程可以作为硬件驱动和 PID 调试工程使用。
- 如果你要跑完整小车业务逻辑，还需要继续整理 `main()` 的主控制流程。

## 3. 软件与构建环境

当前工程按以下环境配置：

- IDE：`Keil MDK`
- 编译器：`Arm Compiler 6 / ArmClang`
- 芯片：`MSPM0G3507`
- SDK：`C:\ti\mspm0_sdk_2_05_01_00`
- SysConfig：`C:\ti\sysconfig_1.24.0`

Keil 工程文件：

- [keil/Car.uvprojx](C:/Users/YunaCelisse/Desktop/Software/Car/keil/Car.uvprojx)

SysConfig 输入文件：

- [empty.syscfg](C:/Users/YunaCelisse/Desktop/Software/Car/empty.syscfg)

SysConfig 生成文件：

- [ti_msp_dl_config.h](C:/Users/YunaCelisse/Desktop/Software/Car/ti_msp_dl_config.h)
- [ti_msp_dl_config.c](C:/Users/YunaCelisse/Desktop/Software/Car/ti_msp_dl_config.c)

## 4. 目录说明

- [empty.c](C:/Users/YunaCelisse/Desktop/Software/Car/empty.c)：主程序入口，当前为 PID 演示
- [Motor.c](C:/Users/YunaCelisse/Desktop/Software/Car/Motor.c)：电机方向与 PWM 输出
- [Encoder.c](C:/Users/YunaCelisse/Desktop/Software/Car/Encoder.c)：编码器中断计数与测速
- [Sensor.c](C:/Users/YunaCelisse/Desktop/Software/Car/Sensor.c)：8 路循迹传感器读取
- [PID.c](C:/Users/YunaCelisse/Desktop/Software/Car/PID.c)：PID 控制器与角度控制
- [speed_loop.c](C:/Users/YunaCelisse/Desktop/Software/Car/speed_loop.c)：速度环任务入口
- [avoid_task.c](C:/Users/YunaCelisse/Desktop/Software/Car/avoid_task.c)：避障任务状态机
- [turn_task.c](C:/Users/YunaCelisse/Desktop/Software/Car/turn_task.c)：转向找线任务状态机
- [h24_task.c](C:/Users/YunaCelisse/Desktop/Software/Car/h24_task.c)：H24 任务
- [pid_test.c](C:/Users/YunaCelisse/Desktop/Software/Car/pid_test.c)：PID 测试任务
- [OLED.c](C:/Users/YunaCelisse/Desktop/Software/Car/OLED.c)：软件 I2C OLED 驱动
- [MPU6050_MSPM0.c](C:/Users/YunaCelisse/Desktop/Software/Car/MPU6050_MSPM0.c)：软件 I2C MPU6050 驱动
- [Ultrasonic.c](C:/Users/YunaCelisse/Desktop/Software/Car/Ultrasonic.c)：超声波测距
- [Key.c](C:/Users/YunaCelisse/Desktop/Software/Car/Key.c)：按键消抖与读取
- [Uart.c](C:/Users/YunaCelisse/Desktop/Software/Car/Uart.c)：串口输出
- [Delay.c](C:/Users/YunaCelisse/Desktop/Software/Car/Delay.c)：SysTick 毫秒时基

## 5. 硬件资源与接线详情

以下接线以“模块信号线 -> MCU 引脚”的方式说明。

### 5.0 供电与共地要求

整车接线前先确认供电架构。

- MCU 核心板工作电压按 `3.3V` 设计。
- OLED、MPU6050 建议直接使用 `3.3V` 供电。
- 编码器、按键、循迹模块的输出电平必须与 `3.3V IO` 兼容。
- 电机和电机驱动的动力电源应与控制电源分开考虑。
- 所有模块必须共地，包括：
  `MSPM0G3507`
  `电机驱动板`
  `编码器`
  `循迹模块`
  `OLED`
  `MPU6050`
  `超声波模块`
  `USB 转串口`

建议供电方式：

- 控制部分：`3.3V`
- 电机驱动部分：按电机额定电压单独供电
- 如果使用开发板板载稳压，不要让电机电流经过 MCU 板载 3.3V 稳压输出

### 5.0.1 高风险电平问题

以下接口如果直接接错，最容易导致 IO 损坏或通信异常。

- 超声波 `Echo` 很多模块输出为 `5V`，必须确认是否已经做了分压。
- 部分循迹模块默认输出跟随供电电压，如果模块工作在 `5V`，要确认数字输出是否为 `3.3V` 兼容。
- 某些 MPU6050 模块带板载上拉到 `3.3V`，也有少数模块把 I2C 上拉到了 `5V`，上电前必须确认。

### 5.1 电机驱动

项目使用两路 PWM + 四路方向控制信号驱动左右电机。

| 功能 | 模块信号 | MCU 引脚 | 说明 |
| --- | --- | --- | --- |
| 左电机 PWM | `PWM_L` | `PB20` | `TIMG12_CCP0` |
| 右电机 PWM | `PWM_R` | `PA25` | `TIMG12_CCP1` |
| 左电机方向 1 | `AIN1` | `PA24` | 电机驱动方向控制 |
| 左电机方向 2 | `AIN2` | `PA16` | 电机驱动方向控制 |
| 右电机方向 1 | `BIN1` | `PB6` | 电机驱动方向控制 |
| 右电机方向 2 | `BIN2` | `PB7` | 电机驱动方向控制 |

接线建议：

- 如果你使用 `TB6612 / MX1508 / L298N` 之类的双路电机驱动板，PWM 接到使能脚，`AIN1/AIN2/BIN1/BIN2` 接方向脚。
- 电机电源与单片机电源必须共地。
- 电机供电不要直接从 MCU IO 取电。

### 5.2 编码器

项目使用两路 AB 相编码器，通过 `GPIOA` 中断统计脉冲。

| 编码器 | 信号 | MCU 引脚 | 说明 |
| --- | --- | --- | --- |
| 左编码器 | `E1A` | `PA15` | GPIO 中断输入 |
| 左编码器 | `E1B` | `PA14` | GPIO 中断输入 |
| 右编码器 | `E2A` | `PA13` | GPIO 中断输入 |
| 右编码器 | `E2B` | `PA12` | GPIO 中断输入 |

说明：

- `Encoder.c` 使用上升沿中断和 AB 相状态判定方向。
- 当前代码为输入上拉模式，适合开漏或普通数字输出型编码器模块。

### 5.3 八路循迹传感器

循迹传感器采用 8 路数字输入，代码中低电平判定为“检测到黑线”。

| 传感器位 | 模块信号 | MCU 引脚 |
| --- | --- | --- |
| S0 | `TRACK_S0` | `PB24` |
| S1 | `TRACK_S1` | `PB14` |
| S2 | `TRACK_S2` | `PB19` |
| S3 | `TRACK_S3` | `PB18` |
| S4 | `TRACK_S4` | `PB17` |
| S5 | `TRACK_S5` | `PB16` |
| S6 | `TRACK_S6` | `PB2` |
| S7 | `TRACK_S7` | `PB9` |

说明：

- [Sensor.c](C:/Users/YunaCelisse/Desktop/Software/Car/Sensor.c) 中 `GPIO_ReadPin()` 对输入做了反相处理。
- 也就是说，模块输出 `0` 时，软件认为该路“有效”。
- 如果你的循迹模块是高电平有效，需要修改 `GPIO_ReadPin()` 逻辑。

### 5.4 OLED 显示屏

OLED 当前使用软件 I2C，物理接线如下：

| OLED 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| `SDA` | `PA0` | 软件 I2C 数据线 |
| `SCL` | `PA1` | 软件 I2C 时钟线 |
| `VCC` | `3.3V` | 建议 3.3V 供电 |
| `GND` | `GND` | 共地 |

说明：

- [OLED.c](C:/Users/YunaCelisse/Desktop/Software/Car/OLED.c) 明确使用 `PA0/PA1` 作为软件 I2C。
- OLED 地址按常见 `0x3C` 设备处理，发送字节为 `0x78`。
- 如果模块板载上拉较弱，I2C 总线可能不稳定，必要时外接上拉电阻。

### 5.5 MPU6050

这里有一个关键注意点。

`SysConfig` 中定义了：

- `MPU_SDA -> PA7`
- `MPU_SCL -> PA6`

但当前 [MPU6050_MSPM0.c](C:/Users/YunaCelisse/Desktop/Software/Car/MPU6050_MSPM0.c) 实际代码并没有使用 `PA6/PA7`，而是复用了 OLED 的软件 I2C 总线：

- `SDA -> PA0`
- `SCL -> PA1`

因此，按当前代码实际行为，MPU6050 应该这样接：

| MPU6050 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| `SDA` | `PA0` | 与 OLED 共用软件 I2C 总线 |
| `SCL` | `PA1` | 与 OLED 共用软件 I2C 总线 |
| `VCC` | `3.3V` | 建议 3.3V |
| `GND` | `GND` | 共地 |
| `AD0` | `GND` | 设备地址为 `0x68` |

必须注意：

- 如果你按照 `PA6/PA7` 去接 MPU6050，当前代码无法正常通信。
- 只有在你修改 `MPU6050_MSPM0.c` 后，`PA6/PA7` 才会生效。
- 当前代码设计是让 OLED 与 MPU6050 共用同一组 I2C 线。

### 5.6 超声波模块

超声波模块使用 1 路触发、1 路回响。

| 超声波信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| `Trig` | `PB8` | 输出触发脉冲 |
| `Echo` | `PB15` | 输入回响脉冲 |
| `VCC` | `5V` 或模块要求 | 视模块型号而定 |
| `GND` | `GND` | 共地 |

说明：

- [Ultrasonic.c](C:/Users/YunaCelisse/Desktop/Software/Car/Ultrasonic.c) 用 GPIO 触发，`TIMG0` 计时。
- 如果你的超声模块 `Echo` 输出是 `5V`，而 MSPM0 IO 不耐 5V，需要加分压或电平转换。

### 5.7 按键

当前定义了 4 个按键输入，按下为低电平有效。

| 按键 | MCU 引脚 |
| --- | --- |
| K1 | `PA22` |
| K2 | `PA27` |
| K3 | `PA28` |
| K4 | `PA31` |

说明：

- [Key.c](C:/Users/YunaCelisse/Desktop/Software/Car/Key.c) 中做了 20ms 消抖。
- 软件按“低电平按下”处理。
- `Key.c` 文件顶部旧注释里提到的 GPIO 位置已经过时，不应作为接线依据。
- 按键实际接线以 [ti_msp_dl_config.h](C:/Users/YunaCelisse/Desktop/Software/Car/ti_msp_dl_config.h) 中 `Key_Pins_*` 定义为准。

### 5.8 串口

当前调试串口为 `UART0`：

| 串口信号 | MCU 引脚 | 波特率 |
| --- | --- | --- |
| `TX` | `PA10` | `115200` |
| `RX` | `PA11` | `115200` |

说明：

- 可接 USB 转串口模块做上位机调试。
- 建议共地。

### 5.9 板载 LED

| 功能 | MCU 引脚 |
| --- | --- |
| LED | `PA2` |

## 6. 定时器与中断分配

| 外设 | 用途 |
| --- | --- |
| `TIMG12` | 左右电机 PWM |
| `TIMA0` | 10ms 周期测速中断 |
| `TIMA1` | 捕获资源，当前工程已初始化 |
| `TIMG0` | 超声波测距计时 |
| `SysTick` | 毫秒时基 |
| `GPIOA_INT_IRQn` | 编码器 AB 相输入中断 |

## 6.1 整车接线总表

这一节用于接线时快速核对，不展开功能说明。

| 模块 | 信号 | MCU 引脚 |
| --- | --- | --- |
| 左电机 PWM | `PWM_L` | `PB20` |
| 右电机 PWM | `PWM_R` | `PA25` |
| 左电机方向 | `AIN1` | `PA24` |
| 左电机方向 | `AIN2` | `PA16` |
| 右电机方向 | `BIN1` | `PB6` |
| 右电机方向 | `BIN2` | `PB7` |
| 左编码器 | `E1A` | `PA15` |
| 左编码器 | `E1B` | `PA14` |
| 右编码器 | `E2A` | `PA13` |
| 右编码器 | `E2B` | `PA12` |
| 循迹 | `S0` | `PB24` |
| 循迹 | `S1` | `PB14` |
| 循迹 | `S2` | `PB19` |
| 循迹 | `S3` | `PB18` |
| 循迹 | `S4` | `PB17` |
| 循迹 | `S5` | `PB16` |
| 循迹 | `S6` | `PB2` |
| 循迹 | `S7` | `PB9` |
| OLED | `SDA` | `PA0` |
| OLED | `SCL` | `PA1` |
| MPU6050 | `SDA` | `PA0` |
| MPU6050 | `SCL` | `PA1` |
| 超声波 | `Trig` | `PB8` |
| 超声波 | `Echo` | `PB15` |
| 按键 | `K1` | `PA22` |
| 按键 | `K2` | `PA27` |
| 按键 | `K3` | `PA28` |
| 按键 | `K4` | `PA31` |
| 串口 | `TX` | `PA10` |
| 串口 | `RX` | `PA11` |
| LED | `LED` | `PA2` |

## 6.2 建议接线顺序

如果你是第一次装车，建议按照下面顺序逐步接入。

1. 只保留下载器和目标板，确认 Keil 可以下载程序。
2. 接 OLED，确认屏幕初始化正常。
3. 接串口，确认 `115200` 下有数据输出。
4. 接编码器，确认中断能进入、速度值会变化。
5. 接电机驱动，但先不要装车轮，确认正反转方向。
6. 接 8 路循迹模块，逐路核对输入有效电平。
7. 接 MPU6050，确认初始化成功。
8. 最后接超声波模块和整车动力部分。

## 7. 当前主程序实际运行流程

这部分补充说明“烧录后现在到底跑什么”。

当前 [empty.c](C:/Users/YunaCelisse/Desktop/Software/Car/empty.c) 的执行流程是：

1. 调用 `SYSCFG_DL_init()`
2. 调用 `System_Init()`
3. 初始化左右轮 PID 参数
4. 进入 `while(1)` 演示循环
5. 用 `fake_speed` 模拟左右轮编码器速度
6. 执行 `PID_control()`
7. 在 OLED 上显示目标值、实际值、PID 输出
8. 每 100ms 刷新一次

这意味着当前程序：

- 不读取实际循迹状态驱动整车
- 不调用完整避障任务
- 不在主循环中使用按键菜单切换运行模式
- 更适合作为电机、编码器、OLED、PID 调试基线

## 7. 软件功能说明

### 7.1 电机控制

- `Set_PWM(int pwm_l, int pwm_r)` 接收左右轮控制量。
- 正负号表示方向。
- 绝对值映射为 `0~100%` 占空比，再换算到 `PWM_PERIOD=1000`。

注意：

- 当前 `Motor.c` 在占空比小于 25 时会强制抬到 25。
- 这会导致即使很小的非零输出也会产生最小驱动力。
- 如果你要更细的低速控制，这一段需要重新调。

### 7.2 编码器测速

- `TIMA0_IRQHandler()` 每 10ms 取一次编码器累计值。
- `leftEncSpeed` 和 `rightEncSpeed` 由 10ms 计数换算得到。
- `Get_Current_Circles()` 可估算圈数。

### 7.3 循迹

- `Sensor_GetState()` 读取 8 路循迹传感器。
- `Sensor_GetQuantizedPos()` 用加权法输出偏差位置。
- 丢线时会更新 `is_lost` 状态。

### 7.4 MPU6050 航向角

- 当前只读取陀螺仪 `Z` 轴。
- 通过积分得到 `yaw_deg`。
- 没有融合加速度计，也没有姿态解算。

因此：

- 适合短时转向控制参考。
- 不适合长时间高精度绝对航向估计。

### 7.5 超声避障

- `Read_Ultrasonic()` 返回距离值。
- `is_obstacle()` 以 `0 < dist < 200` 作为障碍物判定。
- `avoid_task.c` 中有简单避障状态机。

### 7.6 菜单与任务代码

`empty.c` 中已经有菜单状态定义：

- 主菜单
- 圈数设置
- 运行界面
- 避障界面
- MPU6050 调试界面

但当前主循环没有启用这些菜单逻辑，只保留了显示和控制相关代码框架。

## 8. 构建与下载

### 8.1 Keil 构建

1. 打开 [keil/Car.uvprojx](C:/Users/YunaCelisse/Desktop/Software/Car/keil/Car.uvprojx)
2. 确认本机安装：
   `MSPM0 SDK`
   `SysConfig`
   `Keil ArmClang`
3. 直接构建工程

工程在构建前会自动执行 SysConfig：

- 输入：`empty.syscfg`
- 输出：`ti_msp_dl_config.c/.h`

### 8.2 如果构建失败

优先检查下面几项：

- `C:\ti\mspm0_sdk_2_05_01_00` 是否存在
- `C:\ti\sysconfig_1.24.0` 是否存在
- Keil 是否安装了 `MSPM0G3507` 对应 Device Pack
- 编译器是否为 `Arm Compiler 6`

### 8.3 首次编译前检查

第一次在新电脑上打开工程，建议先检查：

1. Keil 中芯片是否能选择 `MSPM0G3507`
2. `Pack Installer` 中是否已安装 TI MSPM0 对应 DFP
3. `C:\ti\mspm0_sdk_2_05_01_00` 是否完整
4. `C:\ti\sysconfig_1.24.0\sysconfig_cli.bat` 是否存在
5. [keil/Car.uvprojx](C:/Users/YunaCelisse/Desktop/Software/Car/keil/Car.uvprojx) 中的 SDK 路径是否与你本机一致

### 8.4 下载与调试建议

- 第一次下载时，先不要接电机动力电源。
- 如果程序能下载但运行异常，先看 OLED 是否亮、串口是否有输出。
- 如果下载器经常断开，优先检查共地和电机电源干扰。

## 9. 上电检查顺序

建议按以下顺序检查硬件：

1. 先只接下载器，确认芯片能下载运行。
2. 接 OLED，确认有显示。
3. 接串口，确认能输出调试信息。
4. 接编码器，确认中断与测速正常。
5. 接电机驱动，但先不装车轮，确认方向和 PWM 正常。
6. 接循迹模块，确认 8 路输入状态正确。
7. 接 MPU6050，确认 `WHO_AM_I` 可读。
8. 最后接超声波和整车机构。

## 9.1 快速故障定位

### 现象 1：程序能下载，但 OLED 不亮

优先检查：

- `PA0/PA1` 是否接反
- OLED 是否为 I2C 版本
- OLED 是否工作在 `3.3V`
- I2C 是否缺少上拉

### 现象 2：电机不转

优先检查：

- 电机驱动板是否单独供电
- `PWM_L/PWM_R` 是否接到了驱动使能脚
- `AIN1/AIN2/BIN1/BIN2` 是否接错
- 电机电源与 MCU 是否共地

### 现象 3：编码器数值一直不变

优先检查：

- `PA12~PA15` 是否接对
- 编码器输出电平是否满足 `3.3V` 输入
- 编码器是否需要外部上拉
- 电机是否真的在转

### 现象 4：MPU6050 初始化失败

优先检查：

- 当前代码实际接的是 `PA0/PA1`，不是 `PA6/PA7`
- `AD0` 是否接地
- 模块供电是否为 `3.3V`
- OLED 与 MPU6050 共总线时是否存在焊接或地址冲突问题

### 现象 5：超声波距离异常

优先检查：

- `Echo` 是否做了电平转换
- 模块前方是否有稳定反射面
- `Trig` 和 `Echo` 是否接反
- 模块供电是否稳定

## 10. 已知问题与注意事项

### 10.1 MPU6050 接线与 SysConfig 不一致

这是当前工程最重要的已知问题。

- `ti_msp_dl_config.h` 里给 MPU6050 配的是 `PA6/PA7`
- 但 `MPU6050_MSPM0.c` 实际用的是 OLED 的 `PA0/PA1`

文档已按“代码真实行为”给出接线说明。

### 10.2 `main()` 不是整车最终逻辑

当前主函数更接近：

- 驱动联调
- PID 参数实验
- OLED 显示验证

不是完整比赛版控制程序。

### 10.3 源码里存在乱码注释

部分中文注释编码异常，但不影响编译。后续如果要长期维护，建议统一转成 `UTF-8`。

### 10.4 超声波 `Echo` 电平兼容

如果超声波模块是 5V 输出，必须确认 `Echo` 进 MCU 前已经降压。

## 11. 建议后续工作

- 整理 `empty.c`，恢复真实整车主控制流程
- 明确 MPU6050 走 `PA0/PA1` 还是 `PA6/PA7`，统一代码和 `SysConfig`
- 给接线定义补一张原理框图或接线图
- 为不同车体版本拆分配置，例如“PID 调试版”和“比赛运行版”

## 12. 文档维护说明

如果后续修改了 [empty.syscfg](C:/Users/YunaCelisse/Desktop/Software/Car/empty.syscfg)，以下内容需要同步更新：

- 接线表
- 定时器与中断分配
- 串口引脚
- OLED 和 MPU6050 总线说明
- README 中的“整车接线总表”

建议原则：

- 引脚分配以 `ti_msp_dl_config.h` 为准
- 但像 MPU6050 这种代码手工复用其他总线的情况，必须同时检查对应 `.c` 文件的真实实现
