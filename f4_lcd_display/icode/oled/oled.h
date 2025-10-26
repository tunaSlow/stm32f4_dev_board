/*
 * oled.h
 *
 *  Created on: Jun 11, 2022
 *      Author: Administrator
 */

#ifndef OLED_OLED_H_
#define OLED_OLED_H_

#include "stm32f4xx_hal.h" //HAL库文件声明
#include "main.h" //IO定义与初始化函数在main.c文件中，必须引用
#include <string.h>//用于字符串处理的库
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

extern I2C_HandleTypeDef hi2c2;//声明I2C2的HAL库结构体

#define OLED0561_ADD	0x78  // OLED的I2C地址（禁止修改）
#define COM				0x00  // OLED 指令（禁止修改）
#define DAT 			0x40  // OLED 数据（禁止修改）

void OLED0561_Init(void);//初始化
void OLED_DISPLAY_ON (void);//OLED屏开显示
void OLED_DISPLAY_OFF (void);//OLED屏关显示
void OLED_DISPLAY_LIT (uint8_t x);//OLED屏亮度设置（0~255）
void OLED_DISPLAY_CLEAR(void);//清屏操作

void OLED_DISPLAY_0816(uint8_t x,uint8_t y,uint16_t w);//显示8x16的单个字符
void OLED_printf_0816_ASCII(uint8_t row,uint8_t column,char *fmt, ...);  //OLED专用的printf函数

void OLED_DISPLAY_1616(uint8_t x,uint8_t y,uint16_t w);
void OLED_printf_1616_CHS(uint8_t row,uint8_t column,char *fmt, ...);

void OLED_DISPLAY_2424(uint8_t x,uint8_t y,uint16_t w);
void OLED_printf_2424_CHS(uint8_t row,uint8_t column,char *fmt, ...);

void OLED_BMP_DISPLAY(uint8_t *BMP);
void OLED_DISPLAY_PIC1(void);
void OLED_DISPLAY_PIC2(void);
void OLED_DISPLAY_GIF(uint8_t SPEED);//显示小车动画（共17帧图片）

#endif /* OLED_OLED_H_ */

/*********************************************************************************************
 * 洋桃电子 www.DoYoung.net
 * 部分程序代码复制自网络开源资料 如有侵权请联系我们处理
 * 洋桃电子原创程序代码部分均未声明版权 可自由复制使用 我们不对代码做任何担保
*********************************************************************************************/
