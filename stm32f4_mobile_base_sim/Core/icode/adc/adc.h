/*
 * adc.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef ICODE_ADC_ADC_H_
#define ICODE_ADC_ADC_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

#define Voltage		3300.0 //����Vref���Ų�����ѹֵ����3300��ʾ3300mV=3.300V������׺�����С�.0����ʹ֮��������

#define ADC_LOOP1_MARK		50 //��һ���˲��Ĳɼ�������������ѡ�ȶ�����10~200���Ƽ�50����ֵ�ϴ�ӦԽ��
#define ADC_LOOP2_MARK		10 //�������˲��Ĳɼ�������������ѡ�ȶ�����10~200���Ƽ�10��

extern uint16_t ADC_DMA_IN[1]; //ADC��ֵ��ŵı���
extern uint16_t ADC_DATA_BUFF1[ADC_LOOP1_MARK];
extern uint16_t ADC_DATA_BUFF2[ADC_LOOP2_MARK];

uint16_t ADC_Read(ADC_HandleTypeDef* hadc);
uint16_t ADCgetavg1(uint16_t adc);
uint16_t ADCgetavg2(uint16_t adc);
uint16_t ADC_to_Voltage(uint16_t adc);

#endif /* ICODE_ADC_ADC_H_ */
