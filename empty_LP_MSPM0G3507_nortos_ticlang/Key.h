#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

void Key_Init(void);

/* 返回：1=检测到“按下事件”(消抖后，上升沿事件)；0=无事件 */
uint8_t Key_GetPressed(uint8_t key_num);

#endif
