# MSPM0 智能小车固件

裸机事件驱动的智能小车固件，包含八路循迹、双编码器速度闭环、MPU6050 航向控制、超声波避障、OLED 菜单和 UART 调试输出。

## 安全提示

- 第一次烧录新控制参数时必须架空车轮。
- K1 为软件急停；运行期间持续扫描，触发后关闭 PWM 并复位控制器。
- 软件急停不能替代切断电机电源或驱动使能的硬件急停。
- IMU、编码器或任务超时会进入故障页并停止电机，K4 清除故障。

## 目录

```text
src/app       主循环、10 ms 调度、故障管理、看门狗
src/control   PID 核心和控制数学
src/drivers   板级驱动
src/tasks     避障和转向状态机
platform/mspm0  SysConfig 配置及生成文件
platform/keil   Keil 工程、启动和分散加载文件
platform/gcc    GCC Makefile及固定生成的链接辅助文件
third_party     工程内固定版本的 TI SDK 依赖
tests/host      可在桌面编译运行的纯算法测试
```

当前只维护 Keil 和 Arm GCC 构建；旧 IAR、TIClang 和重复 Hardware 源码已移除。

## SDK 依赖

工程固定使用 MSPM0 SDK 2.05.01.00，必要的 CMSIS、设备头文件和 MSPM0G1X0X/G3X0X DriverLib 已复制到 `third_party`。普通编译不依赖本机 SDK 或 SysConfig；只有修改 `platform/mspm0/empty.syscfg` 后才需要重新运行 SysConfig。

第三方文件的许可和清单位于：

- `third_party/ti_mspm0_sdk_2_05_01_00/license_mspm0_sdk_2_05_01_00.txt`
- `third_party/ti_mspm0_sdk_2_05_01_00/manifest_mspm0_sdk_2_05_01_00.html`

## Keil 构建

打开：

```text
platform/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
```

工程的头文件和 DriverLib 均使用相对路径。建议执行一次 Rebuild 并确认没有缺失文件或重复中断处理器。

## GCC 构建

安装 Arm GNU Toolchain 后，在 `platform/gcc` 执行：

```powershell
$env:GCC_ARMCOMPILER = 'C:\path\to\arm-gnu-toolchain'
C:\ti\ccs2051\ccs\utils\bin\gmake.exe clean all
```

也可以把 `arm-none-eabi-gcc`、`arm-none-eabi-size` 加入 PATH 后直接运行 `make`。

## 主机测试

安装任一桌面 C 编译器和 CMake 后：

```powershell
cmake -S tests/host -B tests/host/build
cmake --build tests/host/build
ctest --test-dir tests/host/build --output-on-failure
```

## 控制与故障参数

集中配置位于 `src/app/app_config.h`：

- 控制周期：10 ms；
- OLED 刷新：100 ms；
- IMU 连续失败阈值：5 次；
- 转向超时：3 s；
- 寻线超时：5 s；
- 对角直行超时：8 s；
- 编码器堵转判定：有效输出下双轮 500 ms 无脉冲；
- 看门狗：1 s，主循环每 250 ms 在调度心跳正常时喂狗。

固定控制周期后仍保留原 PID 参数和默认速度 80，必须在实车上重新确认稳定性。

## 芯片型号待确认

历史工程名称和 Keil 预处理宏为 `MSPM0G3507`，但当前 SysConfig 使用 `LQFP-48(PT)`，生成文件包含 `CONFIG_MSPM0G3505`。两者属于同一 DriverLib 系列，但封装和具体器件不能靠工程名称推断。烧录前必须核对 MCU 丝印和原理图，然后统一 Keil 宏、SysConfig 器件及链接配置。

## 实车验收

1. 架空车轮，确认上电和故障状态 PWM 为零。
2. 在循迹、避障和转向过程中测试 K1 急停。
3. 分别让超声波 Echo 断线、恒低、恒高，确认 40 ms 超时后可再次测量。
4. 启动前及运行中断开 MPU6050，确认车辆停车并显示故障。
5. 断开编码器或阻止车轮转动，确认 500 ms 内停车。
6. 用逻辑分析仪测量 10 ms 控制周期，目标最坏抖动小于 0.1 ms。
7. 用示波器确认 `Set_PWM(0, 0)`、换向及复位期间的驱动引脚为安全状态。
