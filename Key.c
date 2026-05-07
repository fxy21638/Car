#include "Key.h"
#include "Delay.h"

/* 按键引脚定义来自 SysConfig。
 * 这里统一通过宏引用生成结果，避免手写 GPIO 号和 .syscfg 脱节。 */
#define KEY_PORT (Key_Pins_PORT)
#define KEY0_PIN (Key_Pins_PIN_0_PIN)
#define KEY1_PIN (Key_Pins_PIN_1_PIN)
#define KEY2_PIN (Key_Pins_PIN_2_PIN)
#define KEY3_PIN (Key_Pins_PIN_3_PIN)

#define KEY0_IOMUX (Key_Pins_PIN_0_IOMUX)
#define KEY1_IOMUX (Key_Pins_PIN_1_IOMUX)
#define KEY2_IOMUX (Key_Pins_PIN_2_IOMUX)
#define KEY3_IOMUX (Key_Pins_PIN_3_IOMUX)

/* 去抖时间。
 * 原始电平变化后，必须稳定维持这么久才会被识别为有效事件。 */
#define KEY_DEBOUNCE_MS (20UL)

typedef struct
{
    uint8_t stablePressed;      /* 去抖后的稳定状态：1=按下 */
    uint8_t lastRawPressed;     /* 上一次原始采样状态 */
    unsigned long lastChangeMs; /* 最近一次原始状态变化时间 */
} KeyDebounce;

/* 四个按键各自维护一份去抖状态。 */
static KeyDebounce s_keys[4];

static uint8_t Key_ReadRawPressed(uint8_t key_num)
{
    /* 根据按键编号映射到具体的 pin mask。 */
    uint32_t pinMask;
    switch (key_num)
    {
    case 0:
        pinMask = KEY0_PIN;
        break;
    case 1:
        pinMask = KEY1_PIN;
        break;
    case 2:
        pinMask = KEY2_PIN;
        break;
    case 3:
        pinMask = KEY3_PIN;
        break;
    default:
        return 0;
    }

    /* 硬件约定按下为低电平，因此这里翻转为“1=按下”。 */
    return (DL_GPIO_readPins(KEY_PORT, pinMask) == 0U) ? 1U : 0U;
}

void Key_Init(void)
{
    /* 四个按键都配置为上拉输入：
     * 未按下时为高电平，按下时被拉低。 */
    DL_GPIO_initDigitalInputFeatures(KEY0_IOMUX, DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY1_IOMUX, DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY2_IOMUX, DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY3_IOMUX, DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);

    for (int i = 0; i < 4; i++)
    {
        /* 上电时清空状态机，避免上电抖动被识别成按键事件。 */
        s_keys[i].stablePressed = 0;
        s_keys[i].lastRawPressed = 0;
        s_keys[i].lastChangeMs = tick_ms;
    }
}

uint8_t Key_GetPressed(uint8_t key_num)
{
    if (key_num >= 4)
    {
        return 0;
    }

    KeyDebounce *k = &s_keys[key_num];
    uint8_t rawPressed = Key_ReadRawPressed(key_num);

    /* 原始状态变化时，只记录变化，不立刻上报。 */
    if (rawPressed != k->lastRawPressed)
    {
        k->lastRawPressed = rawPressed;
        k->lastChangeMs = tick_ms;
    }

    /* 如果距离最近一次变化还不到去抖时间，则忽略。 */
    if ((unsigned long)(tick_ms - k->lastChangeMs) < KEY_DEBOUNCE_MS)
    {
        return 0;
    }

    /* 只有稳定状态发生变化时，才认为产生一次有效事件。
     * 当前实现只在“按下”时返回 1，松开不产生事件。 */
    if (rawPressed != k->stablePressed)
    {
        k->stablePressed = rawPressed;
        if (k->stablePressed)
        {
            return 1;
        }
    }

    return 0;
}
