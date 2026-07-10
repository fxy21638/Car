#include "LED.h"

void LED_ON(void)
{
        DL_GPIO_clearPins(LEDB_PORT, LEDB_LED_PIN);
}

void LED_OFF(void)
{
        DL_GPIO_setPins(LEDB_PORT, LEDB_LED_PIN);
}

void LED_Toggle(void)
{
        DL_GPIO_togglePins(LEDB_PORT, LEDB_LED_PIN);
}

void LED_Flash(uint16_t time)
{
	static uint16_t temp;
	if(time==0) LED_ON();
	else if(++temp==time) LED_Toggle(),temp=0;
}
