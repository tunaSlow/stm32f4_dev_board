/*
 * wifi.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef ICODE_WIFI_WIFI_H_
#define ICODE_WIFI_WIFI_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

extern UART_HandleTypeDef huart2;//����UART2��HAL��ṹ��

void WIFI_printf (char *fmt, ...); //WIFIģ�鷢��
void WIFI_TCP_SEND (char *fmt, ...);//��TCPģʽ�µķ������ݣ���������״̬��ä����

#endif /* ICODE_WIFI_WIFI_H_ */
