#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>

/* OLED 字库格式定义 */

#define OLED_CHN_CHAR_WIDTH 2 // UTF-8编码:3字节 GB2312编码:2字节（本工程使用GB2312区位码）

/* 汉字字模结构体 */

typedef struct
{
    char Index[OLED_CHN_CHAR_WIDTH + 1]; // 汉字索引(GB2312内码)
    uint8_t Data[32];                    // 字模数据(16x16像素)
} ChineseCell_t;

/* ASCII字模数据(8x16) */
extern const uint8_t OLED_F8x16[][16];

/* ASCII字模数据(6x8) */
extern const uint8_t OLED_F6x8[][6];

/* 汉字字模数据(16x16) */
extern const ChineseCell_t OLED_CF16x16[];

/* 二极管图标数据(16x16) */
extern const uint8_t Diode[];

/* 其他图形/图像声明 */
//...

#endif

/***************** 协科技|版权所有 ****************/
/***************** jiangxiekeji.com ****************/
