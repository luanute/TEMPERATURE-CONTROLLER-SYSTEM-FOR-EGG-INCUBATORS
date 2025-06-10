#include "rtc.h"
#include "delay_sys.h"
// =================================================================================
// CÁC ĐỊNH NGHĨA VÀ BIẾN CỤC BỘ (CHỈ DÙNG TRONG FILE NÀY)
// =================================================================================
#define EPOCH_YEAR 2020

// Các hàm helper chỉ dùng nội bộ trong file này nên được khai báo là 'static'

 uint32_t DateTimeToSeconds(const Date_t* date, const Time_t* time);
 void SecondsToDateTime(uint32_t total_seconds, Date_t* date, Time_t* time);

// =================================================================================
// ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC (ĐÃ KHAI BÁO EXTERN TRONG .h)
// =================================================================================
volatile Time_t g_currentTime;
volatile Date_t g_currentDate;

// =================================================================================
// CÀI ĐẶT CHI TIẾT CÁC HÀM
// =================================================================================

 uint8_t IsLeapYear(uint16_t year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

 uint8_t GetDaysInMonth(uint8_t month, uint16_t year) {
    const uint8_t daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12) return 0;
    if (month == 2 && IsLeapYear(year)) return 29;
    return daysInMonth[month];
}

 uint32_t DateTimeToSeconds(const Date_t* date, const Time_t* time) {
    uint32_t days = 0;
    uint16_t year_iter;
    for (year_iter = EPOCH_YEAR; year_iter < date->year; year_iter++) {
        days += IsLeapYear(year_iter) ? 366 : 365;
    }
    for (uint8_t m = 1; m < date->month; m++) {
        days += GetDaysInMonth(m, date->year);
    }
    days += date->day - 1;
    return (days * 86400UL) + (time->hours * 3600UL) + (time->minutes * 60UL) + time->seconds;
}

 void SecondsToDateTime(uint32_t total_seconds, Date_t* date, Time_t* time) {
    time->seconds = total_seconds % 60;
    time->minutes = (total_seconds / 60) % 60;
    time->hours   = (total_seconds / 3600) % 24;
    uint32_t total_days = total_seconds / 86400UL;
    uint16_t year = EPOCH_YEAR;
    while(total_days >= (uint16_t)(IsLeapYear(year) ? 366 : 365)) {
        total_days -= IsLeapYear(year) ? 366 : 365;
        year++;
    }
    date->year = year;
    uint8_t month = 1;
    while(total_days >= (uint8_t)GetDaysInMonth(month, year)) {
        total_days -= GetDaysInMonth(month, year);
        month++;
    }
    date->month = month;
    date->day = total_days + 1;
}

void RTC_SetDateTime(const Date_t* date, const Time_t* time) {
    uint32_t counterValue = DateTimeToSeconds(date, time);
    PWR->CR |= PWR_CR_DBP;
    while (((RTC->CRL >> 5U) & 0x01U) != 1U);
    RTC->CRL |= (1U << 4);
    RTC->CNTH = (uint16_t)(counterValue >> 16);
    RTC->CNTL = (uint16_t)(counterValue & 0x0000FFFF);
    RTC->CRL &= ~(1U << 4);
    while (((RTC->CRL >> 5U) & 0x01U) != 1U);
    PWR->CR &= ~PWR_CR_DBP;
}

void RTC_GetDateTime(Date_t* date, Time_t* time) {
    uint32_t counterValue = ((uint32_t)RTC->CNTH << 16) | RTC->CNTL;
    SecondsToDateTime(counterValue, date, time);
}

void RTC_Init(){
		RCC->APB1ENR |=0x18000000U;
		PWR->CR|=0x100U;
		RCC->BDCR|=0x01;
		while(((RCC->BDCR >> 1)& 0x01)!=0x01);
		RCC->BDCR|=0x0100;
		RCC->BDCR|=0x8000;
		
    
		while(((RTC->CRL >> 0x05)& 0x01)!=0x01);
		RTC->CRL|=0x10;
		RTC->PRLH=0;
		RTC->PRLL=0x7FFF;
		RTC->CRH=0x01;
		RTC->CRL &= ~(0x01);
		RTC->CRL &= ~(0x10);
		while(((RTC->CRL >> 0x05)& 0x01)!=0x1);
		NVICx_Init(RTC_IRQn,15,0);
}

void RTC_IRQHandler(void) {
    if (((RTC->CRH >> 0U) & 0x01U) && ((RTC->CRL >> 0U) & 0x01U)) {
        RTC->CRL &= ~(1U << 0);
        RTC_GetDateTime((Date_t*)&g_currentDate, (Time_t*)&g_currentTime);
        // Logic hẹn giờ có thể đặt ở đây
    }
}

