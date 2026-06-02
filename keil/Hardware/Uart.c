#include "Uart.h"

extern int PWMleft, PWMright;
extern int targetLeftSpeed, targetRightSpeed;
extern int leftEncSpeed, rightEncSpeed;

/* ---- 轮询 TX 环形缓冲区 ----
 * 不依赖中断。Uart_PollTx() 在主循环中被高频调用，
 * 将环形缓冲区数据填入 UART TX FIFO。完全无阻塞。 */
#define UART_TX_BUF_SIZE 256
#define UART_TX_BUF_MASK (UART_TX_BUF_SIZE - 1)
static uint8_t  s_txBuf[UART_TX_BUF_SIZE];
static volatile uint16_t s_txHead = 0;
static volatile uint16_t s_txTail = 0;

void Uart_TX_Init(void)
{
    s_txHead = 0;
    s_txTail = 0;
}

/* ---- 主循环轮询发送 ----
 * 注：DL_UART_transmitDataCheck() 在 FIFO 有空间时会顺便发送数据，
 * 因此下面只调用此函数，不再额外调用 transmitData。 */
void Uart_PollTx(void)
{
    while (s_txHead != s_txTail)
    {
        /* transmitDataCheck: FIFO 未满时自动发送 data 并返回 true */
        if (!DL_UART_Main_transmitDataCheck(UART_0_INST,
                                             s_txBuf[s_txHead]))
            break;  /* FIFO 满，下次轮询再试 */
        s_txHead = (s_txHead + 1) & UART_TX_BUF_MASK;
    }
}

/* ---- 入队单字节 ----
 * 写入环形缓冲区。若缓冲区满，阻塞发送一个字节腾出空间（极少发生）。
 * 正常路径下仅操作内存，不触碰 UART。 */
void Uart_SendByte(uint8_t byte)
{
    uint16_t next = (s_txTail + 1) & UART_TX_BUF_MASK;

    while (next == s_txHead)
    {
        /* 缓冲区满（少见）：强制发走一字节腾空间 */
        DL_UART_Main_transmitDataBlocking(UART_0_INST,
                                          s_txBuf[s_txHead]);
        s_txHead = (s_txHead + 1) & UART_TX_BUF_MASK;
    }

    s_txBuf[s_txTail] = byte;
    s_txTail = next;
}

void Uart_SendBytes(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0u) return;
    for (uint32_t i = 0; i < len; i++)
        Uart_SendByte(data[i]);
}

void Uart_SendString(const char *s)
{
    if (s == NULL) return;
    while (*s != '\0')
        Uart_SendByte((uint8_t)(*s++));
}

void Uart_SendInt(int num)
{
    if (num < 0) { Uart_SendByte('-'); num = -num; }
    if (num == 0) { Uart_SendByte('0'); return; }

    char buf[12];
    int i = 0;
    while (num > 0)
    {
        buf[i++] = '0' + (num % 10);
        num /= 10;
    }
    while (--i >= 0)
        Uart_SendByte(buf[i]);
}

/* VOFA+ FireWater 协议：每 50ms 发送一帧速度环数据（20Hz）。
 * 帧格式: TgtL,ActL,TgtR,ActR,PWML,PWMR\r\n */
void VOFA_SendSpeedLoop(void)
{
    static unsigned long lastSendMs = 0;
    unsigned long nowMs = tick_ms;

    if ((nowMs - lastSendMs) < 50UL && lastSendMs != 0)
        return;
    lastSendMs = nowMs;

    /* 手工逐字段发送，避免 snprintf 标准库依赖 */
    Uart_SendInt(targetLeftSpeed);
    Uart_SendByte(',');
    Uart_SendInt(leftEncSpeed);
    Uart_SendByte(',');
    Uart_SendInt(targetRightSpeed);
    Uart_SendByte(',');
    Uart_SendInt(rightEncSpeed);
    Uart_SendByte(',');
    Uart_SendInt(PWMleft);
    Uart_SendByte(',');
    Uart_SendInt(PWMright);
    Uart_SendString("\r\n");
}

void show_dis(void)
{
    static unsigned long lastPrintMs = 0;
    unsigned long nowMs = tick_ms;
    if ((nowMs - lastPrintMs) >= 500UL || lastPrintMs == 0)
    {
        lastPrintMs = nowMs;
        Uart_SendString("hello\r\n");
    }
}
