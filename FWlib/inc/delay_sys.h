#ifndef DELAY_SYS_H
#define DELAY_SYS_H
#include "stm32f10x.h"

extern volatile uint32_t g_sysTickMillis;
void NVICx_Init(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void SysTick_Init(void);
void SysTick_Handler(void);
uint32_t Get_Millis(void);
extern void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t micros(void);
void delayMicroseconds(uint32_t us);

#endif /* DELAY_SYS_H */

