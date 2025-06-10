/*
 * LCD_I2C.c
 *
 * Created on: Apr 8, 2025
 * Author: haidoan2098
 * Modified with standard init sequence and pin mapping guidance
 */

#include "LCD_I2C.h"


//************************** Low Level Function ****************************************************************//
static void CLCD_Delay(uint16_t Time)
{
    delay_ms(Time);  // Giữ lại để đảm bảo tính tương thích với hệ thống
}

static void CLCD_WriteI2C(CLCD_I2C_Name* LCD, uint8_t Data, uint8_t Mode)
{
    char Data_H;
	char Data_L;
	uint8_t Data_I2C[4];
	Data_H = Data&0xF0;
	Data_L = (Data<<4)&0xF0;
	if(LCD->BACKLIGHT)
	{
		Data_H |= LCD_BACKLIGHT; 
		Data_L |= LCD_BACKLIGHT; 
	}
	if(Mode == CLCD_DATA)
	{	
		Data_H |= LCD_RS;
		Data_L |= LCD_RS;
	}
	else if(Mode == CLCD_COMMAND)
	{
		Data_H &= ~LCD_RS;
		Data_L &= ~LCD_RS;
	}
	Data_I2C[0] = Data_H|LCD_EN;
	//delay_us(10);
	Data_I2C[1] = Data_H;
	Data_I2C[2] = Data_L|LCD_EN;
	//delay_us(10);
	Data_I2C[3] = Data_L;
  delay_us(10);
    // Gửi dữ liệu bằng cách sử dụng driver I2C tự viết
    i2c_Start(LCD->I2C, LCD->ADDRESS, 0, 1);
    for (int i = 0; i < 4; i++)
    {
        i2c_sendData(LCD->I2C, Data_I2C[i]);
    }
    i2c_Stop(LCD->I2C);
}

//************************** High Level Function ****************************************************************//
void CLCD_I2C_Init(CLCD_I2C_Name* LCD, I2C_TypeDef* hi2c_CLCD, uint8_t Address, uint8_t Colums, uint8_t Rows)
{
    LCD->I2C = hi2c_CLCD;
    LCD->ADDRESS = Address;
    LCD->COLUMS = Colums;
    LCD->ROWS = Rows;

    LCD->FUNCTIONSET = LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    LCD->ENTRYMODE = LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    LCD->DISPLAYCTRL = LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    LCD->CURSORSHIFT = LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT;
    LCD->BACKLIGHT = LCD_BACKLIGHT;

    CLCD_Delay(50);
    CLCD_WriteI2C(LCD, 0x33, CLCD_COMMAND);  // Khởi tạo 4-bit mode
    //CLCD_WriteI2C(LCD, 0x33, CLCD_COMMAND);
    CLCD_Delay(5);
    CLCD_WriteI2C(LCD, 0x32, CLCD_COMMAND);
    CLCD_Delay(5);
    CLCD_WriteI2C(LCD, 0x20, CLCD_COMMAND);
    CLCD_Delay(5);

    CLCD_WriteI2C(LCD, LCD->ENTRYMODE, CLCD_COMMAND);
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);
    CLCD_WriteI2C(LCD, LCD->CURSORSHIFT, CLCD_COMMAND);
    CLCD_WriteI2C(LCD, LCD->FUNCTIONSET, CLCD_COMMAND);

    CLCD_WriteI2C(LCD, LCD_CLEARDISPLAY, CLCD_COMMAND);
    CLCD_WriteI2C(LCD, LCD_RETURNHOME, CLCD_COMMAND);
}

void CLCD_I2C_SetCursor(CLCD_I2C_Name* LCD, uint8_t Xpos, uint8_t YPos)
{
    uint8_t DRAM_ADDRESS = 0x00;
    if (Xpos >= LCD->COLUMS)
    {
        Xpos = LCD->COLUMS - 1;
    }
    if (YPos >= LCD->ROWS)
    {
        YPos = LCD->ROWS - 1;
    }
    if (YPos == 0)
    {
        DRAM_ADDRESS = 0x00 + Xpos;
    }
    else if (YPos == 1)
    {
        DRAM_ADDRESS = 0x40 + Xpos;
    }
    else if (YPos == 2)
    {
        DRAM_ADDRESS = 0x14 + Xpos;
    }
    else if (YPos == 3)
    {
        DRAM_ADDRESS = 0x54 + Xpos;
    }
    CLCD_WriteI2C(LCD, LCD_SETDDRAMADDR | DRAM_ADDRESS, CLCD_COMMAND);
}

void CLCD_I2C_WriteChar(CLCD_I2C_Name* LCD, char character)
{
    CLCD_WriteI2C(LCD, character, CLCD_DATA);
}

void CLCD_I2C_WriteString(CLCD_I2C_Name* LCD, char *String)
{
    while (*String)
    {
        CLCD_I2C_WriteChar(LCD, *String++);
    }
}

void CLCD_I2C_Clear(CLCD_I2C_Name* LCD)
{
    CLCD_WriteI2C(LCD, LCD_CLEARDISPLAY, CLCD_COMMAND);
    CLCD_Delay(5);
}

void CLCD_I2C_ReturnHome(CLCD_I2C_Name* LCD)
{
    CLCD_WriteI2C(LCD, LCD_RETURNHOME, CLCD_COMMAND);
    CLCD_Delay(5);
}

void CLCD_I2C_CursorOn(CLCD_I2C_Name* LCD)
{
    LCD->DISPLAYCTRL |= LCD_CURSORON;
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);
}

void CLCD_I2C_CursorOff(CLCD_I2C_Name* LCD)
{
    LCD->DISPLAYCTRL &= ~LCD_CURSORON;
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);
}

void CLCD_I2C_BlinkOn(CLCD_I2C_Name* LCD)
{
    LCD->DISPLAYCTRL |= LCD_BLINKON;
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);
}

void CLCD_I2C_BlinkOff(CLCD_I2C_Name* LCD)
{
    LCD->DISPLAYCTRL &= ~LCD_BLINKON;
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);
}

void CLCD_I2C_BacklightOff(CLCD_I2C_Name* LCD)
{
    LCD->BACKLIGHT = 0;  // Tắt đèn nền
    // Gửi lệnh để tắt đèn nền
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);  // Cập nhật lệnh DISPLAYCTRL
}


void CLCD_I2C_BacklightOn(CLCD_I2C_Name* LCD)
{
    LCD->BACKLIGHT = LCD_BACKLIGHT;  // Bật đèn nền (mặc định là 0x08)
    // Gửi lệnh để bật đèn nền
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);
}

void CLCD_I2C_DisplayOn(CLCD_I2C_Name* LCD)
{
    LCD->DISPLAYCTRL |= LCD_DISPLAYON;  // Bật hiển thị màn hình
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);  // Cập nhật lệnh DISPLAYCTRL
}

void CLCD_I2C_DisplayOff(CLCD_I2C_Name* LCD)
{
    LCD->DISPLAYCTRL &= ~LCD_DISPLAYON;  // Tắt hiển thị màn hình
    CLCD_WriteI2C(LCD, LCD->DISPLAYCTRL, CLCD_COMMAND);  // Cập nhật lệnh DISPLAYCTRL
}
