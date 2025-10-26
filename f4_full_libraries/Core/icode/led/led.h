/*
 * led.h
 *
 *  Created on: Oct 22, 2021
 *      Author: Administrator
 */

#ifndef ICODE_LED_LED_H_
#define ICODE_LED_LED_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include "main.h" //IO�������ʼ��������main.c�ļ��У���������

void LED_1(uint8_t a);//LED1�������ƺ�����0ΪϨ������ֵΪ������
void LED_2(uint8_t a);//LED2�������ƺ�����0ΪϨ������ֵΪ������
void LED_3(uint8_t a);//LED3�������ƺ�����0ΪϨ������ֵΪ������
void LED_4(uint8_t a);//LED4�������ƺ�����0ΪϨ������ֵΪ������
void LED_ALL(uint8_t a);//LED1~4���������������4λ��1/0״̬��Ӧ4��LED�������λ��ӦLED1��
void LED_1_Contrary(void);//LED1״̬ȡ��
void LED_2_Contrary(void);//LED2״̬ȡ��
void LED_3_Contrary(void);//LED3״̬ȡ��
void LED_4_Contrary(void);//LED4״̬ȡ��
#endif /* ICODE_LED_LED_H_ */
