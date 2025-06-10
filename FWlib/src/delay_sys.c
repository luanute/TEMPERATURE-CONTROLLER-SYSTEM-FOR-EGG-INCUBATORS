#include "delay_sys.h"

volatile uint32_t g_sysTickMillis = 0;
void NVICx_Init(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
  uint32_t prioritygroup = 0x00U;
  
  prioritygroup = NVIC_GetPriorityGrouping();
  
  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
	NVIC_EnableIRQ(IRQn);
}

// Hàm khởi tạo SysTick để tạo ngắt mỗi 1ms
void SysTick_Init(void) {
    
SysTick->LOAD  = (uint32_t)(SystemCoreClock / 1000) - 1;
SysTick->VAL   = 0UL; 
SysTick->CTRL |= 0x07;

    // Đặt mức ưu tiên ngắt SysTick (tùy chọn)
		NVICx_Init(SysTick_IRQn,2,1);
}

// Trình phục vụ ngắt SysTick (ISR) - được gọi tự động mỗi 1ms
void SysTick_Handler(void) {
    g_sysTickMillis++;
}

// Hàm lấy giá trị mili-giây hiện tại
uint32_t Get_Millis(void) {
    return g_sysTickMillis;
}

// Hàm delay mili-giây
extern void delay_ms(uint32_t ms) {
    uint32_t end  = Get_Millis() + ms;
    while (Get_Millis() - end) {
        // Vòng lặp chờ
    }
}

// Hàm delay micro-giây (sử dụng polling bộ đếm SysTick)
void delay_us(uint32_t us) {
    if (us == 0) return;

    // Ensure SysTick is configured in continuous mode
    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0) return;

    uint32_t ticks_per_us = SystemCoreClock / 1000000;
    uint32_t total_ticks = us * ticks_per_us;
    uint32_t reload_val = SysTick->LOAD;
    uint32_t start_tick = SysTick->VAL;

    uint32_t cur_tick,delta, elapsed = 0;

    while (elapsed < total_ticks) {
        cur_tick = SysTick->VAL;

        if (start_tick >= cur_tick) {
            delta = start_tick - cur_tick;
        } else {// trường hợp đếm tràn
            delta = start_tick + (reload_val - cur_tick + 1);
        }

        elapsed += delta;
        start_tick = cur_tick;
    }
}



uint32_t micros(void) {
    uint32_t ms;
    uint32_t tick_val;
    uint32_t load = SysTick->LOAD + 1; // Tổng số tick trong 1ms
    uint32_t tick_per_us = SystemCoreClock / 1000000;

    do {
        ms = g_sysTickMillis;
        tick_val = SysTick->VAL;
        // Nếu cờ ngắt đã bật trong lúc đọc => đã tràn => cập nhật lại
    } while (ms != g_sysTickMillis);

    // tick_val giảm dần, nên phải trừ ngược lại:
    uint32_t us = (load - tick_val) / tick_per_us;

    return (ms * 1000) + us;
}


void delayMicroseconds(uint32_t us) {
	uint32_t end = micros() + us;
	while (micros() < end);
}

