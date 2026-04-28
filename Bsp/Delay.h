#ifndef _DELAY_H_
#define _DELAY_H_

extern volatile unsigned long tick_ms;

int Delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
void SysTick_Init(void);

#endif  /* #ifndef _DELAY_H_ */
