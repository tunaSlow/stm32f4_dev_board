/*
 * usart1.h
 *
 *  Created on: Oct 20, 2021
 *      Author: Administrator
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

#include "../../Core/icode/tim/tim.h"

extern UART_HandleTypeDef huart1;//����USART1��HAL��ṹ��
extern UART_HandleTypeDef huart2;//����USART2��HAL��ṹ��
extern UART_HandleTypeDef huart3;//����USART3��HAL��ṹ��
extern UART_HandleTypeDef huart4;//����USART4��HAL��ṹ��
extern UART_HandleTypeDef huart6;//����USART4��HAL��ṹ��

#define USART1_REC_LEN  200//����USART1�������ֽ���
#define USART2_REC_LEN  200//����USART2�������ֽ���
#define USART3_REC_LEN  200//����USART3�������ֽ���
#define UART4_REC_LEN  200//����UART4�������ֽ���
#define USART6_REC_LEN  200//����USART3�������ֽ���

extern uint8_t  USART1_RX_BUF[USART1_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART1_RX_STA;//����״̬���
extern uint8_t USART1_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

extern uint8_t  USART2_RX_BUF[USART2_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART2_RX_STA;//����״̬���
extern uint8_t USART2_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

extern uint8_t  USART3_RX_BUF[USART3_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART3_RX_STA;//����״̬���
extern uint8_t USART3_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

extern uint8_t  UART4_RX_BUF[UART4_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t UART4_RX_STA;//����״̬���
extern uint8_t UART4_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

extern uint8_t  USART6_RX_BUF[USART6_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART6_RX_STA;//����״̬���
extern uint8_t USART6_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

void USART1_printf (char *fmt, ...);
void  HAL_UART_RxCpltCallback(UART_HandleTypeDef  *huart);//�����жϻص���������

#endif /* INC_USART_H_ */
