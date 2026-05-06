#include "ti_msp_dl_config.h"
#include "Encoder.h"
#include <stdlib.h>

volatile int32_t Get_Encoder_countA = 0;
volatile int32_t Get_Encoder_countB = 0;
volatile int32_t encoderA_cnt = 0;
volatile int32_t encoderB_cnt = 0;
volatile int32_t totalEncoderCountA = 0;
volatile int32_t totalEncoderCountB = 0;

extern int leftEncSpeed;
extern int rightEncSpeed;

#define PULSE_PER_CIRCLE 26666

void Encoder_Init(void)
{
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoderA_cnt = 0;
    encoderB_cnt = 0;
    totalEncoderCountA = 0;
    totalEncoderCountB = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;

    DL_GPIO_enableInterrupt(GPIOA,
                            ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

void Encoder_ResetDistance(void)
{
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    totalEncoderCountA = 0;
    totalEncoderCountB = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;
}

float Get_Current_Circles(void)
{
    float avg = (abs(totalEncoderCountA) + abs(totalEncoderCountB)) / 2.0f;
    return avg / PULSE_PER_CIRCLE;
}

void TIMA0_IRQHandler(void)
{
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
    uint32_t gpio_interrupt = DL_GPIO_getEnabledInterruptStatus(GPIOA,
                                                                ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                                                    ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if ((gpio_interrupt & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1B_PIN))
        {
            Get_Encoder_countA--;
            totalEncoderCountA--;
        }
        else
        {
            Get_Encoder_countA++;
            totalEncoderCountA++;
        }
    }
    if ((gpio_interrupt & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1A_PIN))
        {
            Get_Encoder_countA++;
            totalEncoderCountA++;
        }
        else
        {
            Get_Encoder_countA--;
            totalEncoderCountA--;
        }
    }

    if ((gpio_interrupt & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2B_PIN))
        {
            Get_Encoder_countB--;
            totalEncoderCountB--;
        }
        else
        {
            Get_Encoder_countB++;
            totalEncoderCountB++;
        }
    }
    if ((gpio_interrupt & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
    {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2A_PIN))
        {
            Get_Encoder_countB++;
            totalEncoderCountB++;
        }
        else
        {
            Get_Encoder_countB--;
            totalEncoderCountB--;
        }
    }

    DL_GPIO_clearInterruptStatus(GPIOA, gpio_interrupt);
}
