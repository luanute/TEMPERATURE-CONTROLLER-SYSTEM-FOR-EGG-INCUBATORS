// main.h hoặc một file header riêng
#ifndef INC_LED7SEG_H_
#define INC_LED7SEG_H_

#include "stm32f10x.h"



// Định nghĩa cổng GPIO chung
#define LED_GPIO_PORT GPIOB

// Định nghĩa SỐ THỨ TỰ của các chân dữ liệu cho segment (0-7 cho PB0-PB7)
#define SEG_A_PIN_NUM  0  // PB0
#define SEG_B_PIN_NUM  1  // PB1
#define SEG_C_PIN_NUM  2  // PB2
#define SEG_D_PIN_NUM  3  // PB3
#define SEG_E_PIN_NUM  4  // PB4
#define SEG_F_PIN_NUM  5  // PB5
#define SEG_G_PIN_NUM  6  // PB6
#define SEG_DP_PIN_NUM 7  // PB7

// Định nghĩa SỐ THỨ TỰ của các chân chọn LED (quét) (12-15 cho PB12-PB15)
#define LED1_PIN_NUM  12 // Hàng chục (ví dụ: 2 trong 27.5C) - PB12
#define LED2_PIN_NUM  13 // Hàng đơn vị (ví dụ: 7 trong 27.5C) - PB13
#define LED3_PIN_NUM  14 // Phần thập phân (ví dụ: 5 trong 27.5C) - PB14
#define LED4_PIN_NUM  15 // Ký tự 'C' - PB15

// Mảng chứa mã hiển thị cho các số từ 0-9 và ký tự 'C' (Common Anode)
// Bit 0 = a, Bit 1 = b, ..., Bit 6 = g, Bit 7 = dp
// Giá trị 0 trong bit tương ứng sẽ làm segment SÁNG.
extern const uint8_t segment_codes[]; // Đưa ra khai báo extern
// Index cho các ký tự đặc biệt
#define CHAR_C_INDEX 10
#define CHAR_DP_INDEX 11 // Không dùng trực tiếp nữa, sẽ xử lý dp riêng

// Buffer chứa các mã segment sẽ hiển thị trên 4 LED
extern uint8_t display_buffer[4]; // LED1, LED2, LED3, LED4
void Timer_Scan_LED_Init(void);
void led7seg_init_gpio_custom(void); // Đổi tên hàm init
void led7seg_set_segments_custom(uint8_t segment_value_with_dp);
void led7seg_select_led_custom(uint8_t led_num);
void led7seg_update_display_custom(float temperature);
void led7seg_scan_custom(void);

#endif /* INC_LED7SEG_H_ */