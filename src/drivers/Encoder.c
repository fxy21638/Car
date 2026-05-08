#include "ti_msp_dl_config.h"
#include "Encoder.h"

volatile int32_t Get_Encoder_countA = 0;
volatile int32_t Get_Encoder_countB = 0;
volatile int32_t encoderA_cnt = 0;
volatile int32_t encoderB_cnt = 0;

extern int leftEncSpeed;
extern int rightEncSpeed;

#define PULSE_PER_CIRCLE 26666

void Encoder_Init(void)
{
    /* 清零计数与测速结果，避免上电后残留脉冲影响控制。 */
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoderA_cnt = 0;
    encoderB_cnt = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;

    /* 使能 A/B 相对应的 GPIO 中断。 */
    DL_GPIO_enableInterrupt(GPIOA,
                            ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

void Encoder_ResetDistance(void)
{
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;
}

float Get_Current_Circles(void)
{
    float avg = (Get_Encoder_countA + Get_Encoder_countB) / 2.0f;
    return avg / PULSE_PER_CIRCLE;
}

void TIMA0_IRQHandler(void)
{
    /* 仅处理定时器归零事件。
     * 每 10ms 把累计脉冲换算为速度，再把累计值清零开始下一周期统计。 */
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        encoderA_cnt = Get_Encoder_countA;
        encoderB_cnt = -Get_Encoder_countB;

        leftEncSpeed = encoderA_cnt * 4;
        rightEncSpeed = encoderB_cnt * 4;

        Get_Encoder_countA = 0;
        Get_Encoder_countB = 0;
    }

    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void GROUP1_IRQHandler(void)
{
    uint32_t gpio_interrupt = DL_GPIO_getEnabledInterruptStatus(
        GPIOA, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                   ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    /* 不能写成 else-if。
     * 同一次中断中，AB 两相都可能同时挂起，必须分别处理。 */
    if ((gpio_interrupt & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1B_PIN))
            Get_Encoder_countA--;
        else
            Get_Encoder_countA++;
    }
    if ((gpio_interrupt & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1A_PIN))
            Get_Encoder_countA++;
        else
            Get_Encoder_countA--;
    }

    if ((gpio_interrupt & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2B_PIN))
            Get_Encoder_countB--;
        else
            Get_Encoder_countB++;
    }
    if ((gpio_interrupt & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2A_PIN))
            Get_Encoder_countB++;
        else
            Get_Encoder_countB--;
    }

    DL_GPIO_clearInterruptStatus(GPIOA, gpio_interrupt);
}
