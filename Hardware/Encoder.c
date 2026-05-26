#include "ti_msp_dl_config.h"
#include "Encoder.h"

// 全局变量
volatile int32_t Get_Encoder_countA = 0; // 左轮实时计数
volatile int32_t Get_Encoder_countB = 0; // 右轮实时计数
volatile int32_t encoderA_cnt = 0;       // 10ms 速度值
volatile int32_t encoderB_cnt = 0;

/* 累计脉冲（用于测距/圈数）。
 * 注意：速度环每 10ms 会把 Get_Encoder_countA/B 清零，所以必须单独累计。
 */
static volatile int32_t s_totalCountA = 0;
static volatile int32_t s_totalCountB = 0;

extern int leftEncSpeed; // 给主程序PID用
extern int rightEncSpeed;

#define PULSE_PER_CIRCLE 26666

// 编码器初始化
void Encoder_Init(void)
{
    // 清空计数
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoderA_cnt = 0;
    encoderB_cnt = 0;
    s_totalCountA = 0;
    s_totalCountB = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;

    // 使能编码器引脚中断
    DL_GPIO_enableInterrupt(GPIOA,
                            ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

// 重置距离
void Encoder_ResetDistance(void)
{
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    s_totalCountA = 0;
    s_totalCountB = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;
}

// 获取圈数
float Get_Current_Circles(void)
{
    /* 右轮在速度计算里做了取反（encoderB_cnt = -Get_Encoder_countB），
     * 这里保持一致：把累计值也转换到“前进为正”的同一符号体系。
     */
    float avg = (s_totalCountA + (-s_totalCountB)) / 2.0f;
    return avg / PULSE_PER_CIRCLE;
}

// 10ms 定时器中断（与官方例程完全一致）
void TIMA0_IRQHandler(void)
{
    /* 读取并判断本次中断源（IIDX）。原来的写法 `if(DL_TIMER_IIDX_ZERO)` 是常量判断，
     * 逻辑上不正确。
     */
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        encoderA_cnt = Get_Encoder_countA;
        encoderB_cnt = -Get_Encoder_countB;

        leftEncSpeed = encoderA_cnt * 2.;
        rightEncSpeed = encoderB_cnt * 2;

        Get_Encoder_countA = 0;
        Get_Encoder_countB = 0;
    }

    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

// GPIOA 组中断处理（编码器正交信号）
void GROUP1_IRQHandler(void)
{
    uint32_t gpio_interrupt = DL_GPIO_getEnabledInterruptStatus(GPIOA,
                                                                ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                                                    ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    // ================== 左编码器 A ==================
    /* 注意：同一轮的 A/B 相位可能在一次 ISR 前后都产生中断并同时置位。
     * 这里不能用 else if，否则会漏处理其中一路。
     */
    if ((gpio_interrupt & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1B_PIN))
        {
            Get_Encoder_countA--;
            s_totalCountA--;
        }
        else
        {
            Get_Encoder_countA++;
            s_totalCountA++;
        }
    }
    if ((gpio_interrupt & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1A_PIN))
        {
            Get_Encoder_countA++;
            s_totalCountA++;
        }
        else
        {
            Get_Encoder_countA--;
            s_totalCountA--;
        }
    }

    // ================== 右编码器 B ==================
    if ((gpio_interrupt & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2B_PIN))
        {
            Get_Encoder_countB--;
            s_totalCountB--;
        }
        else
        {
            Get_Encoder_countB++;
            s_totalCountB++;
        }
    }
    if ((gpio_interrupt & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2A_PIN))
        {
            Get_Encoder_countB++;
            s_totalCountB++;
        }
        else
        {
            Get_Encoder_countB--;
            s_totalCountB--;
        }
    }

    DL_GPIO_clearInterruptStatus(GPIOA, gpio_interrupt);
}
