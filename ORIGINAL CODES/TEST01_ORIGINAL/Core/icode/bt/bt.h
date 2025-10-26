/*
 * bt.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef ICODE_BT_BT_H_
#define ICODE_BT_BT_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include "main.h"
#include "../usart/usart.h"
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

extern UART_HandleTypeDef huart3;//����UART3��HAL��ṹ��

void BT_WEEKUP (void);//����ģ�黽�����������ڿ�����������ʱ���ã�
void BT_SLEEP (void);//����ģ�����˯��ģʽ
void BT_printf (char *fmt, ...); //BT����ģ�鷢��

#endif /* ICODE_BT_BT_H_ */
