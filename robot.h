#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 基础类型定义 — 从各模块提取到此处，打破循环 include
 * ============================================================ */

/* PID 结构体 (原在 PID.h) */
typedef struct {
    float Kp, Ki, Kd;
    float target, actual;
    float err, err_last, err_filtered, err_sum, diff;
    float output, output_max, output_min, sum_max;
    float filter_alpha;
} PID_t;

/* MPU6050 句柄 (原在 MPU6050_MSPM0.h) */
typedef struct {
    float gyroZBias_dps;
    float yaw_deg;
    float gyroZFilt_dps;
    uint32_t lastUpdateMs;
    uint16_t samplePeriodMs;
    int16_t prevRawGyroZ;
} MPU6050_Handle;

/* ============================================================
 * 应用层类型
 * ============================================================ */

typedef enum {
    TASK_TRACE = 0,
    TASK_AVOID,
    TASK_DIAG_1,
    TASK_DIAG_4
} TaskType;

typedef enum {
    MENU_MAIN = 0,
    MENU_TASK_SEL,
    MENU_SET_LAPS,
    MENU_RUNNING,
    MENU_MPU_DEBUG
} MenuState;

typedef struct {
    PID_t leftPID;
    PID_t rightPID;
    PID_t steerPID;
    PID_t anglePID;

    int baseSpeed;
    int baseSpeedTarget;

    int linePos;
    int lastSumPos;
    int isLost;

    float yaw;
    float originYaw;
    MPU6050_Handle imu;
    uint8_t imuOk;

    int16_t dist;

    int pwmLeft;
    int pwmRight;
    int targetLeftSpeed;
    int targetRightSpeed;

    uint8_t running;
    MenuState menuState;
    TaskType taskType;
    uint8_t targetCircles;
    uint8_t lastUserLaps;
} RobotState;

#endif
