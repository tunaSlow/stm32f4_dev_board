/*
 * oled.h
 *
 *  Created on: Jun 11, 2022
 *      Author: Administrator
 */

#ifndef ICODE_OLED_OLED_H_
#define ICODE_OLED_OLED_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include "main.h" //IO�������ʼ��������main.c�ļ��У���������
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

extern I2C_HandleTypeDef hi2c2;//����I2C2��HAL��ṹ��

#define OLED0561_ADD	0x78  // OLED��I2C��ַ����ֹ�޸ģ�
#define COM				0x00  // OLED ָ���ֹ�޸ģ�
#define DAT 			0x40  // OLED ���ݣ���ֹ�޸ģ�

void OLED0561_Init(void);//��ʼ��
void OLED_DISPLAY_ON (void);//OLED������ʾ
void OLED_DISPLAY_OFF (void);//OLED������ʾ
void OLED_DISPLAY_LIT (uint8_t x);//OLED���������ã�0~255��
void OLED_DISPLAY_CLEAR(void);//��������

void OLED_DISPLAY_0816(uint8_t x,uint8_t y,uint16_t w);//��ʾ8x16�ĵ����ַ�
void OLED_printf_0816_ASCII(uint8_t row,uint8_t column,char *fmt, ...);//OLEDר�õ�printf����

void OLED_DISPLAY_1616(uint8_t x,uint8_t y,uint16_t w);
void OLED_printf_1616_CHS(uint8_t row,uint8_t column,char *fmt, ...);

void OLED_DISPLAY_2424(uint8_t x,uint8_t y,uint16_t w);
void OLED_printf_2424_CHS(uint8_t row,uint8_t column,char *fmt, ...);

void OLED_BMP_DISPLAY(uint8_t *BMP);
void OLED_DISPLAY_PIC1(void);



#endif /* ICODE_OLED_OLED_H_ */

/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/
