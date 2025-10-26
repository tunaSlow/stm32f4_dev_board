/*
 * dac.h
 *
 *  Created on: Dec 14, 2022
 *      Author: Administrator
 */

#ifndef ICODE_DAC_DAC_H_
#define ICODE_DAC_DAC_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include "main.h"
#include "../delay/delay.h"

extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim4;

void DAC_Init1(void);//DAC��ʼ��1������ͨ��2����Ƶ�������
void DAC_Init2(void);//DAC��ʼ��2������ͨ��2�����ǲ�������ã���ҪTIM2��MX��Ԥ�����úã�

void DAC_PLAY_WAV1(uint8_t Vol);//DACֱ�Ӳ���ģ����Ƶ��������������1~16��0�Ǿ�����
void DAC_SPEAKER(uint8_t a);

//�ڵ������ǲ����ɺ���֮ǰ��������MX�п���TIM2��DAC������ȷ������ز�������
void DAC_Triangle(void);//DAC���ǲ�����

#endif /* ICODE_DAC_DAC_H_ */
