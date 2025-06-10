#include "led7seg.h"
#include <math.h> // Cho __builtin_ctz hoặc log2 nếu cần, nhưng không cần nữa với PIN_NUM
#include "stm32f1_gpio.h"

// Mảng chứa mã hiển thị cho các số từ 0-9 và ký tự 'C' (Common Anode)
// Bit 0 = a, Bit 1 = b, ..., Bit 6 = g, Bit 7 = dp (dp = 1 là TẮT)
const uint8_t segment_codes[] = {
    0xC0, // 0: ~ (a | b | c | d | e | f) -> dp,g = 1 (tắt); a-f = 0 (sáng)
    0xF9, // 1: ~ (b | c)
    0xA4, // 2: ~ (a | b | d | e | g)
    0xB0, // 3: ~ (a | b | c | d | g)
    0x99, // 4: ~ (b | c | f | g)
    0x92, // 5: ~ (a | c | d | f | g)
    0x82, // 6: ~ (a | c | d | e | f | g)
    0xF8, // 7: ~ (a | b | c)
    0x80, // 8: ~ (a | b | c | d | e | f | g)
    0x90, // 9: ~ (a | b | c | d | f | g)
    0xC6, // C: ~ (a | d | e | f) - Mã này chuẩn hơn cho chữ C hoa
    // Mã 0x7F dùng để bật dấu chấm (AND với nó), không phải là ký tự riêng trong mảng này nữa.
};

// Buffer chứa các mã segment sẽ hiển thị trên 4 LED
uint8_t display_buffer[4];


void Timer_Scan_LED_Init(void) {
    // 1. Bật Clock cho TIM2:
    // Thanh ghi RCC_APB1ENR, bit TIM2EN (bit 0)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Hoặc (1U << 0)

    // 2. Cấu hình Prescaler (PSC) và Auto-Reload Register (ARR):
    // Giả định TIM2CLK là 72MHz. Nếu là 36MHz, PSC = 35.
    TIM2->PSC = 71;    // 72,000,000 Hz / (71 + 1) = 1,000,000 Hz (1MHz -> mỗi tick là 1µs)
    TIM2->ARR = 1999;  // 1,000,000 Hz / (1999 + 1) = 500 Hz (ngắt mỗi 2ms)

    // 3. Cho phép ngắt Update (UIE - Update Interrupt Enable):
    // Thanh ghi TIM2_DIER, bit UIE (bit 0)
    TIM2->DIER |= TIM_DIER_UIE; // Hoặc (1U << 0)

    // 4. (Tùy chọn nhưng khuyến nghị) Bật Auto-Reload Preload Enable (ARPE):
    // Thanh ghi TIM2_CR1, bit ARPE (bit 7)
    // Giúp việc thay đổi ARR (nếu có) được thực hiện một cách an toàn vào cuối chu kỳ update.
    TIM2->CR1 |= TIM_CR1_ARPE;

    // 5. Xóa cờ ngắt Update (UIF) trước khi bật Timer (để đảm bảo không có ngắt giả khi khởi động)
    // Thanh ghi TIM2_SR, bit UIF (bit 0)
    TIM2->SR &= ~TIM_SR_UIF;

    // 6. Bật bộ đếm của Timer (CEN - Counter Enable):
    // Thanh ghi TIM2_CR1, bit CEN (bit 0)
    TIM2->CR1 |= TIM_CR1_CEN;

    // 7. Cấu hình NVIC (Nested Vectored Interrupt Controller) để cho phép ngắt TIM2:
    // Đặt mức ưu tiên cho ngắt TIM2. Giá trị thấp hơn có nghĩa là ưu tiên cao hơn.
    // Mức ưu tiên từ 0 đến 15 cho STM32F10x.
    NVIC_SetPriority(TIM2_IRQn, 1); // Ví dụ: đặt mức ưu tiên là 1 (khá cao, nhưng không phải cao nhất)

    // Cho phép ngắt TIM2 trong NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief Khởi tạo các chân GPIO cho LED 7 đoạn (sử dụng cấu hình HAL ban đầu, không dùng GPIO_Write ở đây).
 * Hàm GPIO_Write của bạn dùng để set/reset, không phải để cấu hình mode/pull/speed.
 */
void led7seg_init_gpio_custom(void) {

				//RCC->APB2ENR |= (1<<0);
				AFIO->MAPR |= (2<<24);
    	  GPIOx_INIT(GPIOB,12, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
        GPIOx_INIT(GPIOB,13, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
				GPIOx_INIT(GPIOB,14, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
		    GPIOx_INIT(GPIOB,15, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
				
				
				GPIOx_INIT(GPIOB,0, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
        GPIOx_INIT(GPIOB,1, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
				GPIOx_INIT(GPIOB,2, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
		    GPIOx_INIT(GPIOB,3, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
				GPIOx_INIT(GPIOB,4, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
        GPIOx_INIT(GPIOB,5, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
				GPIOx_INIT(GPIOB,6, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
		    GPIOx_INIT(GPIOB,7, GPIO_MODE_OUTPUT_PP,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);

    // Ban đầu tắt hết các LED và segment dùng hàm GPIO_Write
    // Tắt LED (kéo lên CAO - State = 1)
    GPIO_Write(LED_GPIO_PORT, LED1_PIN_NUM, 0);
    GPIO_Write(LED_GPIO_PORT, LED2_PIN_NUM, 0);
    GPIO_Write(LED_GPIO_PORT, LED3_PIN_NUM, 0);
    GPIO_Write(LED_GPIO_PORT, LED4_PIN_NUM, 0);

    // Tắt Segments (kéo lên CAO - State = 1)
    //for (uint8_t i = 0; i <= 7; ++i) { // PB0 đến PB7
        GPIO_Write(LED_GPIO_PORT, 0, 1);
				GPIO_Write(LED_GPIO_PORT, 1, 0);
				GPIO_Write(LED_GPIO_PORT, 2, 1);
				GPIO_Write(LED_GPIO_PORT, 3, 1);
				GPIO_Write(LED_GPIO_PORT, 4, 1);
				GPIO_Write(LED_GPIO_PORT, 5, 1);
				GPIO_Write(LED_GPIO_PORT, 6, 0);
				GPIO_Write(LED_GPIO_PORT, 7, 1);
   // }
}

/**
 * @brief Xuất mã hiển thị ra các chân segment bằng hàm GPIO_Write.
 * @param segment_data: Mã 8-bit cho các segment. Bit 0 là segment a, ..., Bit 7 là dp.
 * Giá trị 0 trong bit tương ứng sẽ làm segment SÁNG (kéo xuống THẤP - State = 0).
 * Giá trị 1 trong bit tương ứng sẽ làm segment TẮT (kéo lên CAO - State = 1).
 */
void led7seg_set_segments_custom(uint8_t segment_data) {
    // segment_data có: bit 0 là a, bit 1 là b, ..., bit 7 là dp
    // segment_codes được định nghĩa: 0 là SÁNG, 1 là TẮT
    // Hàm GPIO_Write: State=0 (LOW) là SÁNG, State=1 (HIGH) là TẮT.

    GPIO_Write(LED_GPIO_PORT, SEG_A_PIN_NUM,  (segment_data & (1U << 0)) ? 1 : 0);
    GPIO_Write(LED_GPIO_PORT, SEG_B_PIN_NUM,  (segment_data & (1U << 1)) ? 1 : 0);
    GPIO_Write(LED_GPIO_PORT, SEG_C_PIN_NUM,  (segment_data & (1U << 2)) ? 1 : 0);
    GPIO_Write(LED_GPIO_PORT, SEG_D_PIN_NUM,  (segment_data & (1U << 3)) ? 1 : 0);
    GPIO_Write(LED_GPIO_PORT, SEG_E_PIN_NUM,  (segment_data & (1U << 4)) ? 1 : 0);
    GPIO_Write(LED_GPIO_PORT, SEG_F_PIN_NUM,  (segment_data & (1U << 5)) ? 1 : 0);
    GPIO_Write(LED_GPIO_PORT, SEG_G_PIN_NUM,  (segment_data & (1U << 6)) ? 1 : 0);
    GPIO_Write(LED_GPIO_PORT, SEG_DP_PIN_NUM, (segment_data & (1U << 7)) ? 1 : 0);
}

/**
 * @brief Chọn LED để hiển thị bằng hàm GPIO_Write.
 * @param led_num: Số thứ tự của LED (1 đến 4).
 */
void led7seg_select_led_custom(uint8_t led_num) {
    // Tắt tất cả các LED trước (kéo lên CAO - State = 1)
    GPIO_Write(LED_GPIO_PORT, LED1_PIN_NUM, 1);
    GPIO_Write(LED_GPIO_PORT, LED2_PIN_NUM, 1);
    GPIO_Write(LED_GPIO_PORT, LED3_PIN_NUM, 1);
    GPIO_Write(LED_GPIO_PORT, LED4_PIN_NUM, 1);

    // Bật LED được chọn (kéo xuống THẤP - State = 0)
    switch (led_num) {
        case 1:
            GPIO_Write(LED_GPIO_PORT, LED1_PIN_NUM, 0);
            break;
        case 2:
            GPIO_Write(LED_GPIO_PORT, LED2_PIN_NUM, 0);
            break;
        case 3:
            GPIO_Write(LED_GPIO_PORT, LED3_PIN_NUM, 0);
            break;
        case 4:
            GPIO_Write(LED_GPIO_PORT, LED4_PIN_NUM, 0);
            break;
        default: // Không chọn LED nào (tất cả đều đã tắt)
            break;
    }
}

/**
 * @brief Cập nhật buffer hiển thị dựa trên nhiệt độ.
 * @param temperature: Giá trị nhiệt độ (float).
 */
void led7seg_update_display_custom(float temperature) {
    int temp_int = (int)temperature;
    int temp_dec = (int)((temperature - temp_int) * 10);

    if (temp_int < 0) temp_int = 0;
    if (temp_int > 99) temp_int = 99;
    if (temp_dec < 0) temp_dec = 0;
    if (temp_dec > 9) temp_dec = 9;


    uint8_t tens = temp_int / 10;
    uint8_t units = temp_int % 10;
    uint8_t decimal_digit = temp_dec % 10;

    display_buffer[0] = segment_codes[tens]; // Hàng chục
    display_buffer[1] = segment_codes[units]; // Hàng đơn vị
    display_buffer[2] = segment_codes[decimal_digit]; // Phần thập phân
    display_buffer[3] = segment_codes[CHAR_C_INDEX]; // Ký tự 'C'

    // Bật dấu chấm thập phân cho LED thứ 2 (hàng đơn vị)
    // Mã segment cho dp là bit 7. Để dp SÁNG (0), ta clear bit 7.
    display_buffer[1] &= ~(1U << 7); // Clear bit 7 (AND với 0x7F hay 101111111)
}

/**
 * @brief Thực hiện quét LED.
 */
volatile uint8_t current_led_idx_custom = 0; // Đổi tên biến

void led7seg_scan_custom(void) {
    current_led_idx_custom++;
    if (current_led_idx_custom > 3) {
        current_led_idx_custom = 0;
    }

    // Tắt tất cả các segment trước khi chọn LED mới để tránh "ghosting" nhẹ (tùy chọn)
    // led7seg_set_segments_custom(0xFF); // 0xFF là tất cả segment đều TẮT

    // Đặt dữ liệu cho segment trước khi bật LED
    led7seg_set_segments_custom(display_buffer[current_led_idx_custom]);
    led7seg_select_led_custom(current_led_idx_custom + 1); // led_num từ 1 đến 4
}

