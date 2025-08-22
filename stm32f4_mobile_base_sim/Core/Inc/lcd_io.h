/*
 * lcd_io.h
 *
 *  Created on: Aug 23, 2025
 *      Author: tuna
 */

#ifndef INC_LCD_IO_H_
#define INC_LCD_IO_H_

// lcd_io.h
#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifndef LCD_WIDTH
#define LCD_WIDTH   480
#endif

#ifndef LCD_HEIGHT
#define LCD_HEIGHT  800   // Change to 272 if your panel is 480x272
#endif
// Adjust these if your RS (D/C) is NOT on A16:
#define LCD_REG16   (*((__IO uint16_t *)(0x60000000)))  // RS=0 (command)
#define LCD_RAM16   (*((__IO uint16_t *)(0x60020000)))  // RS=1 (data) assuming A16

// GPIO you assign in CubeMX/user code:
extern GPIO_TypeDef* LCD_RESET_GPIO_Port;
extern uint16_t      LCD_RESET_Pin;
extern GPIO_TypeDef* LCD_BL_GPIO_Port;
extern uint16_t      LCD_BL_Pin;

void LCD_FMC_Init(void);     // call after MX_FMC_Init()
void LCD_Reset(void);
void LCD_Backlight_On(uint8_t on);
void LCD_WriteCmd(uint8_t cmd);
void LCD_WriteData8(uint8_t data);
void LCD_WriteData16(uint16_t data);
void LCD_WriteDataMulti(const uint8_t* buf, uint32_t len);
void LCD_Delay(uint32_t ms);
void LCD_DisplayOn_Minimal(void);  // 0x11, 0x29, 0x3A=RGB565
void LCD_FillRGB565(uint16_t color);


#endif /* INC_LCD_IO_H_ */
