/*
 * rs485.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef ICODE_RS485_RS485_H_
#define ICODE_RS485_RS485_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

extern UART_HandleTypeDef huart4;//����UART4��HAL��ṹ��
void RS485_printf (char *fmt, ...);  //RS485����

#endif /* ICODE_RS485_RS485_H_ */
