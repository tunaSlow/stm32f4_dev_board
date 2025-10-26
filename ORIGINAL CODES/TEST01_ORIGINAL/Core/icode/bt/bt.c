/*
 * bt.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/bt/bt.h"


void BT_WEEKUP (void)//����ģ�黽�����������ڿ�����������ʱ���ã�
{
	HAL_GPIO_WritePin(BTCS_GPIO_Port,BTCS_Pin, GPIO_PIN_RESET);//PWRC�ӿڿ���
	HAL_Delay(10);//��ʱ
}
void BT_SLEEP (void)//����ģ�����˯��ģʽ
{
	HAL_GPIO_WritePin(BTCS_GPIO_Port,BTCS_Pin, GPIO_PIN_SET);//PWRC�ӿڿ���
	HAL_Delay(10);//��ʱ
}
//����ģ��ͨ�ţ�ʹ��UART3������BT������printf����
//���÷�����BT_printf("123"); //��UART3�����ַ�123
void BT_printf (char *fmt, ...)
{
    char buff[USART3_REC_LEN+1];  //���ڴ��ת��������� [����]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, USART3_REC_LEN+1,fmt,arg_ptr);//����ת��
    i=strlen(buff);//�ó����ݳ���
    if(strlen(buff)>USART3_REC_LEN)i=USART3_REC_LEN;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
    HAL_UART_Transmit(&huart3,(uint8_t *)buff,i,0xffff);//���ڷ��ͺ��������ںţ����ݣ����������ʱ�䣩
    va_end(arg_ptr);
}
//����USART���ڵ��жϻص�����HAL_UART_RxCpltCallback��ͳһ����ڡ�USART1.C���ļ��С�
