#include "Encoder.h"

// 全局变量（与官方例程格式一致）
volatile int32_t Get_Encoder_countA = 0; // 左轮实时计数
volatile int32_t Get_Encoder_countB = 0; // 右轮实时计数
volatile int32_t encoderA_cnt = 0;       // 10ms 速度值
volatile int32_t encoderB_cnt = 0;

/* 累计脉冲（不清零），用于 Get_Current_Circles() 圈数计算。
 * Get_Encoder_countA/B 每 10ms 被 TIMA0 ISR 清零，不能用于累计距离。 */
static volatile int32_t s_totalCountA = 0;
static volatile int32_t s_totalCountB = 0;

extern int leftEncSpeed; // 给主程序PID用
extern int rightEncSpeed;

static uint32_t s_pulsesPerCircle = 13000;

// 编码器初始化
void Encoder_Init(void)
{
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoderA_cnt = 0;
    encoderB_cnt = 0;
    s_totalCountA = 0;
    s_totalCountB = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;

    DL_GPIO_enableInterrupt(GPIOA,
                            ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

void Encoder_ResetDistance(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    s_totalCountA = 0;
    s_totalCountB = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;
    __set_PRIMASK(primask);
}

void Encoder_GetSnapshot(EncoderSnapshot *snapshot)
{
    uint32_t primask;
    if (snapshot == NULL)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->totalLeft = s_totalCountA;
    snapshot->totalRight = s_totalCountB;
    snapshot->speedLeft = leftEncSpeed;
    snapshot->speedRight = rightEncSpeed;
    __set_PRIMASK(primask);
}

float Get_Current_Circles(void)
{
    EncoderSnapshot snapshot;
    float avg;
    Encoder_GetSnapshot(&snapshot);
    avg = (snapshot.totalLeft - snapshot.totalRight) / 2.0f;
    return avg / (float)s_pulsesPerCircle;
}

int32_t Encoder_GetDistancePulses(void)
{
    EncoderSnapshot snapshot;
    Encoder_GetSnapshot(&snapshot);
    return (snapshot.totalLeft - snapshot.totalRight) / 2;
}

/* ---- 速度采样 ISR (10ms 周期) ----
 * 读取编码器正交计数 → 差分速度 → 供 PID_control 速度环使用。
 * 右轮取反 (encoderB_cnt = -Get_Encoder_countB) 保证前进时为正。 */
void Encoder_SampleSpeed10msFromISR(void)
{
    encoderA_cnt = Get_Encoder_countA;
    encoderB_cnt = -Get_Encoder_countB;

    leftEncSpeed = encoderA_cnt * 4;   /* 脉冲×4 → 速度 (10ms周期) */
    rightEncSpeed = encoderB_cnt * 4;

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
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

void Encoder_SetPulsesPerCircle(uint32_t pulses)
{
    s_pulsesPerCircle = pulses;
}
