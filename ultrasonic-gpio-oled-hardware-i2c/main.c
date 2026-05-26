#include "ti_msp_dl_config.h"
#include "main.h"
#include "stdio.h"

uint16_t distVal = 0;

uint8_t oled_buffer[32];

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    // MPU6050_Init();
    OLED_Init();
    Ultrasonic_Init();

    /* Don't remove this! */
    Interrupt_Init();

    OLED_ShowString(0,0,(uint8_t *)"Dist:",16);

    while (1) 
    {
        distVal = Read_Ultrasonic();
        sprintf((char *)oled_buffer, "%4u", distVal);
        OLED_ShowString(6*8,0,oled_buffer,16);
        mspm0_delay_ms(200);
    }
}
