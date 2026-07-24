#include "PID.h"

/**
 * @brief  PID初始化
 * @param  pid: PID结构体指针
 * @param  Kp: 比例系数
 * @param  Ki: 积分系数
 * @param  Kd: 微分系数
 * @param  output_max: 输出最大值
 * @param  output_min: 输出最小值
 * @param  sum_max: 积分限幅（建议为误差范围的1/5~1/3）
 * @param  filter_alpha: 低通滤波系数（0.1~0.3，推荐0.2）
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
             float output_max, float output_min, float sum_max, float filter_alpha) {
    // 初始化参数
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->sum_max = sum_max;
    pid->filter_alpha = (filter_alpha > 1.0f) ? 1.0f :
                       (filter_alpha < 0.0f) ? 0.0f : filter_alpha;

    // 初始化状态变量
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_filtered = 0.0f;
    pid->err_sum = 0.0f;
    pid->diff = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  PID更新（核心控制逻辑）
 * @note   在定时中断中调用（如10ms一次）
 * @param  pid: PID结构体指针
 */
void PID_Update(PID_t *pid) {
    // 1. 计算当前误差（未滤波）
    pid->err = pid->target - pid->actual;

    // 2. 一阶低通滤波（抑制高频噪声）
    pid->err_filtered = pid->filter_alpha * pid->err + (1 - pid->filter_alpha) * pid->err_filtered;

    // 3. 计算微分（滤波后的误差变化率）
    pid->diff = pid->err_filtered - pid->err_last;
    pid->err_last = pid->err_filtered; // 保存当前滤波误差供下次计算

    // 4. 计算积分（积分分离：输出超限时停止积分）
    if (!(pid->output >= pid->output_max && pid->err_filtered > 0) &&
        !(pid->output <= pid->output_min && pid->err_filtered < 0)) {
        pid->err_sum += pid->err_filtered;
        // 积分限幅（防止积分饱和）
        if (pid->err_sum > pid->sum_max) {
            pid->err_sum = pid->sum_max;
        } else if (pid->err_sum < -pid->sum_max) {
            pid->err_sum = -pid->sum_max;
        }
    }

    // 5. 计算PID输出
    pid->output = pid->Kp * pid->err_filtered +  // 比例项（滤波后误差，抗震荡）
                 pid->Ki * pid->err_sum +        // 积分项（消除稳态误差）
                 pid->Kd * pid->diff;            // 微分项（抑制超调）

    // 6. 输出限幅（确保PWM有效范围）
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
}

/**
 * @brief  重置PID状态（急停或切换模式时调用）
 * @param  pid: PID结构体指针
 */
void PID_Reset(PID_t *pid) {
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_filtered = 0.0f;
    pid->err_sum = 0.0f;
    pid->diff = 0.0f;
    pid->output = 0.0f;
}
