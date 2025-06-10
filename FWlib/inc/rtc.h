#ifndef ___RTC_H__
#define ___RTC_H__
#include "stm32f10x.h"
#include <stdbool.h>   

// --- Định nghĩa các cấu trúc Time_t và Date_t để các file khác có thể sử dụng ---
typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} Time_t;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
} Date_t;

// --- Khai báo các biến toàn cục mà main.c và các file khác cần truy cập ---
// 'extern' báo cho trình biên dịch biết rằng các biến này được định nghĩa ở một file .c khác
extern volatile Time_t g_currentTime;
extern volatile Date_t g_currentDate;

// --- Khai báo các hàm công khai (Public Functions) ---
// Đây là các hàm mà main.c sẽ gọi để tương tác với RTC.

/**
 * @brief  Khởi tạo RTC, cấu hình LSE, prescaler, và ngắt mỗi giây.
 * Kiểm tra Backup Register để tránh cấu hình lại nếu RTC đã chạy.
 * @param  None
 * @retval None
 */
void RTC_Init(void);

/**
 * @brief  Đặt ngày và giờ cho RTC.
 * Hàm này sẽ tính tổng số giây từ EPOCH và ghi vào bộ đếm RTC.
 * @param  date: Con trỏ đến cấu trúc Date_t chứa ngày tháng cần đặt.
 * @param  time: Con trỏ đến cấu trúc Time_t chứa thời gian cần đặt.
 * @retval None
 */
void RTC_SetDateTime(const Date_t* date, const Time_t* time);

/**
 * @brief  Lấy ngày và giờ hiện tại từ RTC.
 * Hàm này sẽ đọc bộ đếm RTC và chuyển đổi thành định dạng ngày/tháng/năm giờ:phút:giây.
 * @param  date: Con trỏ đến cấu trúc Date_t để lưu kết quả ngày tháng.
 * @param  time: Con trỏ đến cấu trúc Time_t để lưu kết quả thời gian.
 * @retval None
 */
void RTC_GetDateTime(Date_t* date, Time_t* time);
 uint32_t DateTimeToSeconds(const Date_t* date, const Time_t* time);
 void SecondsToDateTime(uint32_t total_seconds, Date_t* date, Time_t* time);
 uint8_t IsLeapYear(uint16_t year);
 uint8_t GetDaysInMonth(uint8_t month, uint16_t year);
#endif // __RTC_H