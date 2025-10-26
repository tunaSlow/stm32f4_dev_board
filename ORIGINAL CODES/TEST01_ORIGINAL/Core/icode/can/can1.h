/*
 * can.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef ICODE_CAN_CAN1_H_
#define ICODE_CAN_CAN1_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

#include "../usart/usart.h" //���õ�USART1_printf�ڳ����ն���ʾCAN״̬��Ϣ

extern CAN_HandleTypeDef hcan1;//������HAL��ṹ��

extern CAN_TxHeaderTypeDef     TxMeg;//CAN����������ؽṹ��
extern CAN_RxHeaderTypeDef     RxMeg;//CAN����������ؽṹ��

#define CAN1_ID_H      0x0000 //32λ����ID���ã���16λ��
#define CAN1_ID_L      0x0000 //32λ����ID���ã���16λ��
#define CAN1_MASK_H    0x0000 //32λ����MASK���ã���16λ��
#define CAN1_MASK_L    0x0000 //32λ����MASK���ã���16λ��
#define CAN1_REC_LEN  200//����CAN1�������ֽ���

#define CAN1_HOST_ID      0x12 //CAN���߱��豸ID��ֻҪ���������豸��ͻ�����������ã�

extern uint8_t  CAN1_RX_BUF[CAN1_REC_LEN];//���ջ���,���CAN1_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t CAN1_RX_STA;//����״̬���

void CAN_User_Init(CAN_HandleTypeDef* hcan  );//CAN�û���ʼ������
void  HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);//CAN���ջص�����
uint8_t  CAN1_SendNormalData(CAN_HandleTypeDef*  hcan,uint16_t ID,uint8_t *pData,uint16_t  Len);//CAN���ͺ���
void CAN1_printf (char *fmt, ...);//CAN����ͨ�ţ�ʹ��CAN1������CANר�õ�printf����

#endif /* ICODE_CAN_CAN1_H_ */
