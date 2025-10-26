/*
 * adc.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/adc/adc.h"

uint16_t ADC_DATA_BUFF1[ADC_LOOP1_MARK];//ADC��ֵ���
uint16_t ADC_DATA_BUFF2[ADC_LOOP2_MARK];//ADC��ֵ���

uint16_t ADC_Read(ADC_HandleTypeDef* hadc){ //ADC�ɼ����򣨲�����ADC�ľ����
	HAL_ADC_Start(hadc);//��ʼADC�ɼ�
	HAL_ADC_PollForConversion(hadc,500);//�ȴ��ɼ�����
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(hadc), HAL_ADC_STATE_REG_EOC)){//��ȡADC��ɱ�־λ
		return HAL_ADC_GetValue(hadc);//����ADC��ֵ
	}
	return 0xFFFF;//����ȡADCʧ��ʱ�򷵻�0xFFFF
}

//----------------------������ƽ��ֵ�˲��㷨����-----------------------//

uint16_t ADCgetavg1(uint16_t adc){ //����һ��ADC�˲�����1��ADCֵ�����������ݿ⣬ȥ�������Сֵ����С100�����õ��ȶ����ݡ�
	uint16_t maxA,minA;
	uint32_t avg;
	uint8_t jA;
	avg=0;//��ʼ״̬ADCֵΪ0
	maxA=0;minA=0xffff;//���������С��ʼֵ����ֹ��������Сֵʱ����
	for(jA=0;jA<(ADC_LOOP1_MARK-1);jA++){//��ADC�������ݼĴ���������λ��������1�飬�Կճ�[0]λ����µ�ֵ
		ADC_DATA_BUFF1[(ADC_LOOP1_MARK-1)-jA] = ADC_DATA_BUFF1[(ADC_LOOP1_MARK-2)-jA];//�Ĵ���������λ��������1��
	}
	ADC_DATA_BUFF1[0]=adc;//[0]λ����µ�ADCֵ
	for(jA=0;jA<ADC_LOOP1_MARK;jA++){//���Ĵ����е������Сֵ������maxA��minA��
		if(maxA<=ADC_DATA_BUFF1[jA])maxA=ADC_DATA_BUFF1[jA];//�����ֵ
		if(minA>=ADC_DATA_BUFF1[jA])minA=ADC_DATA_BUFF1[jA];//����С��
	}
	for(jA=0;jA<ADC_LOOP1_MARK;jA++){//��ȥ�����Сֵ֮�����
		if(ADC_DATA_BUFF1[jA]==maxA)maxA=0xffff;//������ֵ���֣���������ֵ����ֹ���ֵ��ͬʱ�ظ�����
		if(ADC_DATA_BUFF1[jA]==minA)minA=0;//�����Сֵ���֣��������Сֵ����ֹ���ֵ��ͬʱ�ظ�����
		if(ADC_DATA_BUFF1[jA]!=maxA && ADC_DATA_BUFF1[jA]!=minA){//��ֵ���������Сֵʱִ��
			avg+=ADC_DATA_BUFF1[jA];//�����ֵ���
		}
	}
	minA=avg/ADC_LOOP1_MARK;//��Ӻ��ֵȥ����2λ���õ��ȶ���ֵ
	return  minA;          //����ƽ��ֵ
}

uint16_t ADCgetavg2(uint16_t adc){ //���ڶ���ADC�˲�����10��һ���˲�������ݣ�ֻ��10�ζ���ͬ�ŷ������յ�������ֵ
	uint8_t jA;
	uint8_t cou=0;
	for(jA=1;jA<(ADC_LOOP2_MARK-1);jA++){//��ADC�������ݼĴ���������λ��������1�飬�Կճ�[1]λ����µ�ֵ��ע��[ADC_LOOP2_MARK-1]λ���ڴ�����շ�����ֵ��
		ADC_DATA_BUFF2[(ADC_LOOP2_MARK-1)-jA] = ADC_DATA_BUFF2[(ADC_LOOP2_MARK-2)-jA];//�Ĵ���������λ��������1��
	}
	ADC_DATA_BUFF2[0]=adc;//[0]λ����µ�ADCֵ
	for(jA=0;jA<=ADC_LOOP2_MARK-2;jA++){//�ȶԴ�[0]��[ADC_LOOP2_MARK-2]
		if(ADC_DATA_BUFF2[jA]==adc){//�ȶ������Ƿ���ͬ
			cou++;//ÿ��ͬһ�Σ�������1
			if(cou>=ADC_LOOP2_MARK-2){//�����ж��������ݶ���ͬ
				ADC_DATA_BUFF2[ADC_LOOP2_MARK-1] = adc;//���������ݷ���
				return adc;//���������ݷ���
			}
		}else{//���ȶ����ݲ�ͬʱ
			cou=0;//������0
		}
	}
	return ADC_DATA_BUFF2[ADC_LOOP2_MARK-1];//���ȶ����ݲ�ͬʱ����֮ǰδ���µ�����
}

uint16_t ADC_to_Voltage(uint16_t adc){//ADC��ѹ���㣨������ADCֵ�����أ���ѹֵ����λ��mV��
	return Voltage/4095*adc;//Voltage�ǵ�Ƭ��Vref���Ų�����ѹ����adc.h�ļ����ã�4095��12λADC���ֵ������
}

/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/





