/*
 * tim.c
 *
 *  Created on: Apr 23, 2022
 *      Author: Administrator
 */

#include "../../Core/icode/tim/tim.h"
TIM_HandleTypeDef htim2;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //��ʱ���жϻص�����
{
   if(htim ==&htim2)//�ж��Ƿ��Ƕ�ʱ��2�жϣ���ʱ����ʱ��ʾһ���ַ������ս�����
    {
		USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//��ӽ�����
		USART2_RX_STA|=0x8000;//���ձ�־λ���λ��1��ʾ�������
		__HAL_TIM_CLEAR_FLAG(&htim2,TIM_EVENTSOURCE_UPDATE );//���TIM2�����жϱ�־
		__HAL_TIM_DISABLE(&htim2);//�رն�ʱ��2
    }
}


