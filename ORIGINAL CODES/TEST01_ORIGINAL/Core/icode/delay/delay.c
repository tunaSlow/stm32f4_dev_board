/*
 * delay.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/delay/delay.h"

void delay_us(uint32_t us) //����CPUѭ��ʵ�ֵķǾ�׼Ӧ�õ�΢����ʱ����
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 8000000 * us); //ʹ��HAL_RCC_GetHCLKFreq()������ȡ��Ƶֵ�����㷨�õ�1΢���ѭ������
    while (delay--); //ѭ��delay�Σ��ﵽ1΢����ʱ
}
