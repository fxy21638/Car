#include "ti_msp_dl_config.h"
#include "Encoder.h"
#include "Uart.h"
#include "Delay.h"

static int s_leftEncSpeed  = 0;
static int s_rightEncSpeed = 0;

int Encoder_GetLeftSpeed(void)  { return s_leftEncSpeed; }
int Encoder_GetRightSpeed(void) { return s_rightEncSpeed; }

static volatile int32_t s_totalCountA = 0;
static volatile int32_t s_totalCountB = 0;
static uint32_t s_pulsesPerCircle = 13000;
volatile uint8_t g_mpu6050_flag = 0;

void Encoder_Init(void)
{
    s_totalCountA   = 0;
    s_totalCountB   = 0;
    s_leftEncSpeed  = 0;
    s_rightEncSpeed = 0;
    g_mpu6050_flag  = 0;

    DL_GPIO_enableInterrupt(GPIOA,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
            ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

void Encoder_ResetDistance(void)
{
    s_totalCountA   = 0;
    s_totalCountB   = 0;
    s_leftEncSpeed  = 0;
    s_rightEncSpeed = 0;
}

float Get_Current_Circles(void)
{
    float avg = (s_totalCountA + (-s_totalCountB)) / 2.0f;
    return avg / (float)s_pulsesPerCircle;
}

int32_t Encoder_GetDistancePulses(void)
{
    return (int32_t)((s_totalCountA - s_totalCountB) / 2);
}

void Encoder_SetPulsesPerCircle(uint32_t pulses)
{
    s_pulsesPerCircle = pulses;
}

void GROUP1_IRQHandler(void)
{
    uint32_t stat = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
            ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if (stat & ENCODERA_E1A_PIN) {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1B_PIN)) s_totalCountA--;
        else                                             s_totalCountA++;
    }
    if (stat & ENCODERA_E1B_PIN) {
        if (!DL_GPIO_readPins(GPIOA, ENCODERA_E1A_PIN)) s_totalCountA++;
        else                                             s_totalCountA--;
    }
    if (stat & ENCODERB_E2A_PIN) {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2B_PIN)) s_totalCountB--;
        else                                             s_totalCountB++;
    }
    if (stat & ENCODERB_E2B_PIN) {
        if (!DL_GPIO_readPins(GPIOA, ENCODERB_E2A_PIN)) s_totalCountB++;
        else                                             s_totalCountB--;
    }

    DL_GPIO_clearInterruptStatus(GPIOA, stat);
}

void TIMA0_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        static int32_t s_lastA = 0, s_lastB = 0;
        int32_t curA = s_totalCountA;
        int32_t curB = s_totalCountB;
        s_leftEncSpeed  = curA - s_lastA;
        s_rightEncSpeed = -(curB - s_lastB);
        s_lastA = curA;
        s_lastB = curB;
        g_mpu6050_flag = 1;
    }
    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void Encoder_DebugPrint(void)
{
    static unsigned long lastPrintMs = 0;
    unsigned long nowMs = tick_ms;
    if ((nowMs - lastPrintMs) < 200UL && lastPrintMs != 0) return;
    lastPrintMs = nowMs;
    Uart_SendString("LA:");  Uart_SendInt((int)s_totalCountA);
    Uart_SendString(" LB:"); Uart_SendInt((int)s_totalCountB);
    Uart_SendString(" SA:"); Uart_SendInt(s_leftEncSpeed);
    Uart_SendString(" SB:"); Uart_SendInt(s_rightEncSpeed);
    Uart_SendString(" D:");  Uart_SendInt((int)Encoder_GetDistancePulses());
    Uart_SendString("\r\n");
}
