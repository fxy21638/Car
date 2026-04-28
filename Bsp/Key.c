#include "Key.h"
#include "Delay.h"

/* ======================== 引脚配置（来自 SysConfig） ========================
 * 使用 ti_msp_dl_config.h 里生成的 Key_Pins_* 宏：
 * - Key0: Key_Pins_PIN_0_* (GPIOB.10)
 * - Key1: Key_Pins_PIN_1_* (GPIOB.8)
 * - Key2: Key_Pins_PIN_2_* (GPIOB.7)
 * - Key3: Key_Pins_PIN_3_* (GPIOB.6)
 * 按下为低电平。
 */
#define KEY_PORT (Key_Pins_PORT)
#define KEY0_PIN (Key_Pins_PIN_0_PIN)
#define KEY1_PIN (Key_Pins_PIN_1_PIN)
#define KEY2_PIN (Key_Pins_PIN_2_PIN)
#define KEY3_PIN (Key_Pins_PIN_3_PIN)

#define KEY0_IOMUX (Key_Pins_PIN_0_IOMUX)
#define KEY1_IOMUX (Key_Pins_PIN_1_IOMUX)
#define KEY2_IOMUX (Key_Pins_PIN_2_IOMUX)
#define KEY3_IOMUX (Key_Pins_PIN_3_IOMUX)

#define KEY_DEBOUNCE_MS (20UL)

typedef struct
{
    uint8_t stablePressed;
    uint8_t lastRawPressed;
    unsigned long lastChangeMs;
} KeyDebounce;

static KeyDebounce s_keys[4];

static uint8_t Key_ReadRawPressed(uint8_t key_num)
{
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

    /* 按下为低电平 */
    return (DL_GPIO_readPins(KEY_PORT, pinMask) == 0U) ? 1U : 0U;
}

void Key_Init(void)
{
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

    if (rawPressed != k->lastRawPressed)
    {
        k->lastRawPressed = rawPressed;
        k->lastChangeMs = tick_ms;
    }

    if ((unsigned long)(tick_ms - k->lastChangeMs) < KEY_DEBOUNCE_MS)
    {
        return 0;
    }

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
