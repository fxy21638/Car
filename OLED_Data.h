#ifndef __OLED_DATA_H

#define __OLED_DATA_H

#include <stdint.h>

/*ַֽڿ*/

#define OLED_CHN_CHAR_WIDTH 2 // UTF-8ʽ3GB2312ʽ2

/*ģԪ*/

typedef struct

{

    char Index[OLED_CHN_CHAR_WIDTH + 1]; //

    uint8_t Data[32]; // ģ

} ChineseCell_t;

/*ASCIIģ*/

extern const uint8_t OLED_F8x16[][16];

extern const uint8_t OLED_F6x8[][6];

/*ģ*/

extern const ChineseCell_t OLED_CF16x16[];

/*ͼ*/

extern const uint8_t Diode[];

/*ĸʽλüµͼ*/

//...

#endif

/*****************ЭƼ|Ȩ****************/

/*****************jiangxiekeji.com*****************/
