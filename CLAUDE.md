# CLAUDE.md — MSPM0G3507 智能小车项目

## 项目概述

基于 TI MSPM0G3507 (ARM Cortex-M0+, 80MHz) 的循线智能小车，运行在 LP_MSPM0G3507 LaunchPad 上。无 RTOS，采用超级循环 (super-loop) 架构。代码为历史遗留，不再维护，仅供参考。

## 目录结构

```
car_02/
  empty.c                  # 主入口，超级循环+菜单状态机
  empty.syscfg             # SysConfig 项目文件（引脚/外设配置的唯一来源）
  ti_msp_dl_config.c/.h    # SysConfig 自动生成的初始化代码

  Hardware/                # [主要] 跨构建共享的驱动模块
    Motor.c/h              #   PWM 电机驱动 (TIMG12, -100~+100)
    Sensor.c/h             #   8路红外循线传感器 (加权位置 -79~+79)
    Encoder.c/h            #   编码器速度测量 (TIMA0 10ms采样)
    MPU6050_MSPM0.c/h      #   MPU6050 陀螺仪 (软件I2C, Z轴偏航积分)
    Ultrasonic.c/h         #   HC-SR04 超声波 (50us ISR, 非阻塞)
    OLED.c/h               #   SSD1306 OLED 128x64 (软件I2C, 帧缓冲)
    OLED_Data.c/h          #   OLED 字库和图像数据
    PID.c/h                #   通用PID算法 (低通滤波+积分分离+输出限幅)
    Key.c/h                #   4键按钮 (软件去抖)
    LED.c/h                #   板载LED
    Delay.c/h              #   SysTick 毫秒定时
    Uart.c/h               #   UART0 调试输出 (115200)

  keil/                    # Keil MDK 构建 (ARMCLANG V6.21, 最完整)
    Hardware/              #   扩展模块: task.c/h (避障/转向状态机),
                           #   PID.c 含 PID_control/TurnToAngle 等完整控制函数
    empty_LP_MSPM0G3507_nortos_keil.uvprojx

  gcc/                     # GCC (arm-none-eabi-gcc) 构建
  iar/                     # IAR (iccarm) 构建
  ticlang/                 # TI Clang (tiarmclang) 构建
  empty_LP_MSPM0G3507_nortos_ticlang/  # CCS/Theia 构建目录
  ultrasonic-gpio-oled-hardware-i2c/   # 独立子项目 (超声波OLED显示)
```

## 架构

```
main() 超级循环:
  1. Sensor_GetQuantizedPos() → 通过8路红外计算加权线位置
  2. 超声波/MPU6050 传感器更新
  3. Key_GetPressed() 处理菜单输入
  4. 根据 g_menuState 和 g_running 选择控制模式:
     - PID_control()       循线PID (steerPID → leftPID/rightPID → Set_PWM)
     - ObstacleAvoidance_Task()  6阶段避障状态机
     - turn_Task()         4阶段脱线恢复状态机 (MPU6050偏航角辅助)
  5. OLED 显示更新 (非阻塞)

中断:
  SysTick_Handler          (1ms)   → tick_ms++
  TIMA0_IRQHandler         (10ms)  → 编码器速度采样
  GROUP1_IRQHandler        (GPIOA) → 编码器正交解码
  TIMER_US_INST_IRQHandler (50us)  → 超声波回波测量
```

## 4个PID控制器

| 控制器 | 用途 | 位置 |
|--------|------|------|
| `steerPID` | 循线转向 (输入: linePos, 输出: 速度差) | empty.c |
| `leftPID` / `rightPID` | 左右轮速度闭环 | empty.c |
| `anglePID` | IMU航向角控制 (脱线恢复时使用) | empty.c |

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

## 编码

- 源文件编码: **UTF-8** (部分文件原为GBK，已转换)
- 注释语言: 中文
- 缩进: 4空格

## 注意事项

1. **SysConfig 是引脚配置的唯一来源**。修改 `empty.syscfg` 后重新生成 `ti_msp_dl_config.c/.h`。
2. **根 Hardware/ 和 keil/Hardware/ 存在差异**。keil 版本包含完整控制函数 (PID_control, TurnToAngle) 和任务层 (task.c)。
3. **主循环无固定周期**。代码中注释掉了 `Delay_ms(10)`，实际运行频率取决于各模块耗时。
4. **OLED 和 MPU6050 共用软件 I2C 总线**，需注意访问冲突。
5. **`ultrasonic-gpio-oled-hardware-i2c/` 是独立子项目**，有自己的 main.c，不参与主构建。
