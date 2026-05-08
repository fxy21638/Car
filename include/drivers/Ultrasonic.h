#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

void Ultrasonic_Init(void);
int16_t Read_Ultrasonic(void);
int is_obstacle(int16_t dist);

#endif
