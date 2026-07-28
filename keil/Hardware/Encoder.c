/*
 * ============================================================
 *  encoder — 正交编码器实现
 *
 *  GROUP1_IRQHandler: A/B 相四倍频解码 (任意边沿中断 → 状态机)
 *
 *  TIMA0_IRQHandler: 每 10ms 计算速度 (mm/s) + 置 MPU6050 标志
 * ============================================================
 */

#include "Encoder.h"
#include "Uart.h"
#include "Delay.h"

/* ---- 编码器标定（四倍频解码，需根据实车测量后填入） ---- */
static uint32_t s_leftCountsPerM  = 6896L;   /* 左轮每米脉冲数 (6cm轮径, 1300/圈, 4x) */
static uint32_t s_rightCountsPerM = 6896L;   /* 右轮每米脉冲数 (6cm轮径, 1300/圈, 4x) */
static int      s_leftSign        = 1;        /* 左轮速度符号: +1 或 -1 */
static int      s_rightSign       = 1;        /* 右轮速度符号: +1 或 -1 */

static int  s_leftSpeed_mms;   /* 左轮速度 mm/s */
static int  s_rightSpeed_mms;  /* 右轮速度 mm/s */
static uint32_t s_pulsesPerCircle = 13000;

volatile int32_t g_encoder_left_count;
volatile int32_t g_encoder_right_count;
volatile uint8_t g_mpu6050_flag;

int Encoder_GetLeftSpeed(void)  { return s_leftSpeed_mms; }
int Encoder_GetRightSpeed(void) { return s_rightSpeed_mms; }

void Encoder_Init(void)
{
    g_encoder_left_count  = 0;
    g_encoder_right_count = 0;
    s_leftSpeed_mms  = 0;
    s_rightSpeed_mms = 0;
    g_mpu6050_flag   = 0;

    /* 四倍频解码：A/B 相全部双边沿中断 */
    DL_GPIO_enableInterrupt(GPIOB, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                   ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    /* 设置全部四路为双边沿触发（SysConfig 只配了上升沿，此处覆盖） */
    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_3_EDGE_RISE_FALL | DL_GPIO_PIN_2_EDGE_RISE_FALL | DL_GPIO_PIN_15_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_16_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
                                        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
}

void Encoder_ResetDistance(void)
{
    g_encoder_left_count  = 0;
    g_encoder_right_count = 0;
    s_leftSpeed_mms  = 0;
    s_rightSpeed_mms = 0;
}

void Encoder_Read(int32_t *left, int32_t *right)
{
    __disable_irq();
    *left  = g_encoder_left_count;
    *right = g_encoder_right_count;
    __enable_irq();
}

float Get_Current_Circles(void)
{
    int32_t l, r;
    Encoder_Read(&l, &r);
    float avg = (float)(l + r) / 2.0f;
    return avg / (float)s_pulsesPerCircle;
}

int32_t Encoder_GetDistancePulses(void)
{
    int32_t l, r;
    Encoder_Read(&l, &r);
    return (l + r) / 2;
}

void Encoder_SetPulsesPerCircle(uint32_t pulses)
{
    s_pulsesPerCircle = pulses;
}

void Encoder_SetLeftCountsPerM(uint32_t counts)  { s_leftCountsPerM = counts; }
void Encoder_SetRightCountsPerM(uint32_t counts) { s_rightCountsPerM = counts; }
void Encoder_SetLeftSign(int sign)               { s_leftSign = sign; }
void Encoder_SetRightSign(int sign)              { s_rightSign = sign; }

/* 四倍频正交解码状态迁移表
   state = (A<<1 | B): 0=00, 1=01, 2=10, 3=11
   quad_table[prev][cur] = 脉冲增量 (+1顺时针, -1逆时针, 0无效跳变) */
static const int8_t s_quad_table[4][4] = {
    /*cur: 0   1   2   3   prev: */
    {      0, +1, -1,  0 },  /* 0: A=0,B=0 */
    {     -1,  0,  0, +1 },  /* 1: A=0,B=1 */
    {     +1,  0,  0, -1 },  /* 2: A=1,B=0 */
    {      0, -1, +1,  0 },  /* 3: A=1,B=1 */
};

/* 四倍频解码 ISR: A/B 相任意边沿触发，查表判方向，每周期 ±4 脉冲 */
void GROUP1_IRQHandler(void)
{
    static uint8_t s_prevL = 0;  /* 左轮上一次 A:B 状态 */
    static uint8_t s_prevR = 0;  /* 右轮上一次 A:B 状态 */

    uint32_t stat = DL_GPIO_getEnabledInterruptStatus(GPIOB,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN |
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if (stat & (ENCODERA_E1A_PIN | ENCODERA_E1B_PIN)) {
        uint8_t a = DL_GPIO_readPins(GPIOB, ENCODERA_E1A_PIN) ? 1U : 0U;
        uint8_t b = DL_GPIO_readPins(GPIOB, ENCODERA_E1B_PIN) ? 1U : 0U;
        uint8_t cur = (a << 1) | b;
        g_encoder_left_count += s_quad_table[s_prevL][cur];
        s_prevL = cur;
    }
    if (stat & (ENCODERB_E2A_PIN | ENCODERB_E2B_PIN)) {
        uint8_t a = DL_GPIO_readPins(GPIOB, ENCODERB_E2A_PIN) ? 1U : 0U;
        uint8_t b = DL_GPIO_readPins(GPIOB, ENCODERB_E2B_PIN) ? 1U : 0U;
        uint8_t cur = (a << 1) | b;
        g_encoder_right_count += s_quad_table[s_prevR][cur];
        s_prevR = cur;
    }

    DL_GPIO_clearInterruptStatus(GPIOB, stat);
}

/* 每 10ms: 计算增量 → mm/s 速度 + 置 MPU6050 标志 */
void TIMA0_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        static int32_t s_lastL, s_lastR;
        int32_t curL = g_encoder_left_count;
        int32_t curR = g_encoder_right_count;
        int32_t dL  = curL - s_lastL;
        int32_t dR  = curR - s_lastR;
        s_lastL = curL;
        s_lastR = curR;

        /* mm/s = delta * 100000 (10ms→s) / COUNTS_PER_M
         * s_leftSign/s_rightSign 由 Encoder_SetLeftSign/SetRightSign 配置，
         * 默认为 +1。若某轮实测速度方向反了，调整对应符号即可。 */
        s_leftSpeed_mms  = (int)((s_leftSign  * dL * 100000L) / (int32_t)s_leftCountsPerM);
        s_rightSpeed_mms = (int)((s_rightSign * dR * 100000L) / (int32_t)s_rightCountsPerM);

        g_mpu6050_flag = 1;
    }
    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
}

void Encoder_DebugPrint(void)
{
    static unsigned long lastPrintMs;
    unsigned long nowMs = tick_ms;
    if ((nowMs - lastPrintMs) < 200UL && lastPrintMs) return;
    lastPrintMs = nowMs;

    Uart_SendString("L:");  Uart_SendInt((int)g_encoder_left_count);
    Uart_SendString(" R:"); Uart_SendInt((int)g_encoder_right_count);
    Uart_SendString(" SL:"); Uart_SendInt(s_leftSpeed_mms);
    Uart_SendString(" SR:"); Uart_SendInt(s_rightSpeed_mms);
    Uart_SendString("\r\n");
}
