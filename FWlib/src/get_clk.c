#include "get_clk.h"

uint32_t Get_HCLK_Frequency(void) {
	/*HCLK (AHB Clock - Bus tốc độ cao, dùng cho CPU và DMA).
     PCLK1 (APB1 Clock - Bus tốc độ thấp, dùng cho các ngoại vi như TIM2, UART2,..).
     PCLK2 (APB2 Clock - Bus tốc độ cao, dùng cho các ngoại vi như GPIO, ADC, TIM1,..).*/
	 
    uint32_t sysclk, hclk;
    uint32_t pllmul, pllsource;
     
    // Lấy giá trị SW (System Clock Switch Status)
    uint32_t sws = (RCC->CFGR >> 2) & 0x03;

    if (sws == 0) {
        sysclk = 8000000;  // HSI = 8MHz
    } else if (sws == 1) {
        sysclk = 8000000;  // HSE = 8MHz (cần kiểm tra nếu có crystal khác)
    } else if (sws == 2) {
        // PLL được chọn làm System Clock
        pllsource = (RCC->CFGR >> 16) & 0x01; // 0: HSI/2 được chọn làm nguồn đầu vào PLL
		                                      // 1: HSE được chọn làm nguồn đầu vào PLL
        pllmul = ((RCC->CFGR >> 18) & 0x0F) + 2;
		                                      /*0010: PLL input clock x 4
												0011: PLL input clock x 5
												0100: PLL input clock x 6
												0101: PLL input clock x 7
												0110: PLL input clock x 8
												0111: PLL input clock x 9
												10xx: Reserved
												1100: Reserved
												1101: PLL input clock x 6.5
												111x: Reserved*/

        if (pllsource == 0) {
            sysclk = (8000000 / 2) * pllmul;  // HSI/2 làm nguồn PLL
        } else {
            sysclk = (8000000 * pllmul);  // HSE làm nguồn PLL
        }
    }

    // Lấy Prescaler AHB (HPRE)
    uint32_t hpre = (RCC->CFGR >> 4) & 0x0F;
    if (hpre >= 8) {
        hclk = sysclk >> (hpre - 7);
    } else {
        hclk = sysclk;
    }                                     /*0xxx: SYSCLK not divided
											1000: SYSCLK divided by 2
											1001: SYSCLK divided by 4
											1010: SYSCLK divided by 8
											1011: SYSCLK divided by 16
											1100: SYSCLK divided by 64
											1101: SYSCLK divided by 128
											1110: SYSCLK divided by 256
											1111: SYSCLK divided by 512*/

    return hclk;
}

uint32_t Get_PCLK1_Frequency(void) {
    uint32_t hclk = Get_HCLK_Frequency();
    uint32_t ppre1 = (RCC->CFGR >> 8) & 0x07;

    if (ppre1 >= 4) {
        return hclk >> (ppre1 - 3);
    } else {
        return hclk;  //max PCLK1 36MHz
    }                 /*0xx: HCLK not divided
						100: HCLK divided by 2
						101: HCLK divided by 4
						110: HCLK divided by 8
						111: HCLK divided by 16*/
}

uint32_t Get_PCLK2_Frequency(void) {
    uint32_t hclk = Get_HCLK_Frequency();
    uint32_t ppre2 = (RCC->CFGR >> 11) & 0x07;

    if (ppre2 >= 4) {
        return hclk >> (ppre2 - 3);
    } else {
        return hclk;
    }         /*0xx: HCLK not divided
				100: HCLK divided by 2
				101: HCLK divided by 4
				110: HCLK divided by 8
				111: HCLK divided by 16*/
}
// get PCLK1 and PCLK2


