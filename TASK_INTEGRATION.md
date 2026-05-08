# Task Integration

本文档说明当前任务文件的拆分方式，以及新任务如何接入主循环。

## 当前结构

- [empty.c](/c:/Users/YunaCelisse/Desktop/Software/Car/empty.c)
  负责系统初始化、MPU 更新、主循环调度。
- [speed_loop.c](/c:/Users/YunaCelisse/Desktop/Software/Car/speed_loop.c)
  只保留 `SpeedLoop_Task()`。
- [avoid_task.c](/c:/Users/YunaCelisse/Desktop/Software/Car/avoid_task.c)
  只保留 `ObstacleAvoidance_Task()`。
- [turn_task.c](/c:/Users/YunaCelisse/Desktop/Software/Car/turn_task.c)
  只保留 `turn_Task()`。
- [h24_task.c](/c:/Users/YunaCelisse/Desktop/Software/Car/h24_task.c)
  保留 `H24` 任务的完整逻辑，包括按键、OLED、状态机和运行控制。
- [pid_test.c](/c:/Users/YunaCelisse/Desktop/Software/Car/pid_test.c)
  保留 `PID` 测试任务的完整逻辑。
- [app_state.h](/c:/Users/YunaCelisse/Desktop/Software/Car/app_state.h)
  统一声明跨任务共享的运行状态和 PID/IMU 全局对象，减少各任务文件内部分散的 `extern`。

## 主循环接入方式

当前主循环在 [empty.c](/c:/Users/YunaCelisse/Desktop/Software/Car/empty.c) 中只保留一行任务调用：

```c
H24_Task(nowMs);
/* PIDTest_Task(nowMs); */
```

这表示：

- 想运行 `H24`，直接调用 `H24_Task(nowMs)`。
- 想运行 `PID` 测试，直接改成 `PIDTest_Task(nowMs)`。
- 不需要在 `main()` 中再单独处理该任务的按键、菜单、OLED 或状态切换。

## 头文件边界

- [speed_loop.h](/c:/Users/YunaCelisse/Desktop/Software/Car/speed_loop.h)
  只声明 `SpeedLoop_Task()`。
- [avoid_task.h](/c:/Users/YunaCelisse/Desktop/Software/Car/avoid_task.h)
  只声明 `ObstacleAvoidance_Task()`。
- [turn_task.h](/c:/Users/YunaCelisse/Desktop/Software/Car/turn_task.h)
  只声明 `turn_Task()`。
- [h24_task.h](/c:/Users/YunaCelisse/Desktop/Software/Car/h24_task.h)
  只声明 `H24_Task()`。
- [pid_test.h](/c:/Users/YunaCelisse/Desktop/Software/Car/pid_test.h)
  只声明 `PIDTest_Task()`。

这样拆分后，每个任务各自拥有独立的 `.c/.h`，不再保留一个汇总式的 `task.c`。
共享状态则统一从 [app_state.h](/c:/Users/YunaCelisse/Desktop/Software/Car/app_state.h) 引入。

## 新任务接入规则

如果后续新增一个完整任务，例如 `MyTask`，建议按下面方式接入：

1. 新建独立文件，例如 `my_task.c` / `my_task.h`。
2. 在 `my_task.c` 内封装完整任务逻辑：
   按键、菜单、OLED、状态机、运行控制都放进去。
3. 只对外暴露一个主入口，例如：

```c
void MyTask(unsigned long nowMs);
```

4. 在 [empty.c](/c:/Users/YunaCelisse/Desktop/Software/Car/empty.c) 主循环中直接一行调用：

```c
MyTask(nowMs);
```

## 现有任务说明

- `H24` 内部使用数字任务号 `k0`、`k1`、`k2`、`k3`。
  这是按题目小题编号保留的，不改成枚举名称。
- `ObstacleAvoidance_Task()` 和 `turn_Task()` 仍保留原函数名，可继续正常调用。
- `ObstacleAvoidance_Reset()` 和 `TurnTask_Reset()` 已补齐，后续如果做任务切换，可以先显式复位状态机。
- 工程文件 [keil/Car.uvprojx](/c:/Users/YunaCelisse/Desktop/Software/Car/keil/Car.uvprojx) 已加入 [h24_task.c](/c:/Users/YunaCelisse/Desktop/Software/Car/h24_task.c) 和 [pid_test.c](/c:/Users/YunaCelisse/Desktop/Software/Car/pid_test.c)。
