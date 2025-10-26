/*
 * key.c
 *
 *  Created on: Oct 22, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/key/key.h"


uint8_t KEY_1(void)
{
	uint8_t a;
	a=0;//���δ���밴�������򷵻�0
	if(HAL_GPIO_ReadPin(GPIOF,KEY1_Pin)==GPIO_PIN_RESET){//�������ӿڵĵ�ƽ
		HAL_Delay(20);//��ʱȥ����
		if(HAL_GPIO_ReadPin(GPIOF,KEY1_Pin)==GPIO_PIN_RESET){ //�������ӿڵĵ�ƽ
			a=1;//���밴����������1
		}
	}
	while(HAL_GPIO_ReadPin(GPIOF,KEY1_Pin)==GPIO_PIN_RESET); //�ȴ������ɿ�
	return a;
}

uint8_t KEY_2(void)
{
	uint8_t a;
	a=0;//���δ���밴�������򷵻�0
	if(HAL_GPIO_ReadPin(GPIOF,KEY2_Pin)==GPIO_PIN_RESET){//�������ӿڵĵ�ƽ
		HAL_Delay(20);//��ʱȥ����
		if(HAL_GPIO_ReadPin(GPIOF,KEY2_Pin)==GPIO_PIN_RESET){ //�������ӿڵĵ�ƽ
			a=1;//���밴����������1
		}
	}
	while(HAL_GPIO_ReadPin(GPIOF,KEY2_Pin)==GPIO_PIN_RESET); //�ȴ������ɿ�
	return a;
}

uint8_t KEY_3(void)
{
	uint8_t a;
	a=0;//���δ���밴�������򷵻�0
	if(HAL_GPIO_ReadPin(GPIOF,KEY3_Pin)==GPIO_PIN_RESET){//�������ӿڵĵ�ƽ
		HAL_Delay(20);//��ʱȥ����
		if(HAL_GPIO_ReadPin(GPIOF,KEY3_Pin)==GPIO_PIN_RESET){ //�������ӿڵĵ�ƽ
			a=1;//���밴����������1
		}
	}
	while(HAL_GPIO_ReadPin(GPIOF,KEY3_Pin)==GPIO_PIN_RESET); //�ȴ������ɿ�
	return a;
}

uint8_t KEY_4(void)
{
	uint8_t a;
	a=0;//���δ���밴�������򷵻�0
	if(HAL_GPIO_ReadPin(GPIOF,KEY4_Pin)==GPIO_PIN_RESET){//�������ӿڵĵ�ƽ
		HAL_Delay(20);//��ʱȥ����
		if(HAL_GPIO_ReadPin(GPIOF,KEY4_Pin)==GPIO_PIN_RESET){ //�������ӿڵĵ�ƽ
			a=1;//���밴����������1
		}
	}
	while(HAL_GPIO_ReadPin(GPIOF,KEY4_Pin)==GPIO_PIN_RESET); //�ȴ������ɿ�
	return a;
}

/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/
