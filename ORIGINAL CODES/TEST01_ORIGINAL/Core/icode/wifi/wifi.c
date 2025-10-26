/*
 * wifi.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/wifi/wifi.h"

#include "main.h"
#include "../../Core/icode/usart/usart.h"

//WIFIģ��ͨ�ţ�ʹ��UART2������ר�õ�printf����
//���÷�����WIFI_printf("123"); //��USART2�����ַ�123
void WIFI_printf (char *fmt, ...)
{
	char buff[USART2_REC_LEN+1];  //���ڴ��ת��������� [����]
	uint16_t i=0;
	va_list arg_ptr;
	va_start(arg_ptr, fmt);
	vsnprintf(buff, USART2_REC_LEN+1, fmt, arg_ptr);//����ת��
	i=strlen(buff);//�ó����ݳ���
	if(strlen(buff)>USART2_REC_LEN)i=USART2_REC_LEN;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
    HAL_UART_Transmit(&huart2,(uint8_t *)buff,i,0xffff);//���ڷ��ͺ��������ںţ����ݣ����������ʱ�䣩
    va_end(arg_ptr);
}
//WIFIģ����TCPģʽ�µ����ݷ��ͣ�TCP���͵Ĺ涨���ȷ�AT+CIPSEND=�������ȴ����ء�>�����ٷ����������ݡ�
//���÷�����WIFI_TCP_SEND("123\r\n"); //TCP��ʽ�����ַ�123�ͻس�����
void WIFI_TCP_SEND (char *fmt, ...)
{
	char buff[USART2_REC_LEN+1];  //���ڴ��ת��������� [����]
	uint16_t i=0;
	va_list arg_ptr;
	va_start(arg_ptr, fmt);
	vsnprintf(buff, USART2_REC_LEN+1, fmt, arg_ptr);//����ת��
	i=strlen(buff);//�ó����ݳ���
	if(strlen(buff)>USART2_REC_LEN)i=USART2_REC_LEN;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
	WIFI_printf("AT+CIPSEND=%d\r\n",i);//�ȷ���ATָ�����������
	HAL_Delay(100);//�ȴ�WIFIģ�鷵��">"���˴�û�������ǲ���">"���жϡ��ȶ���Ҫ��ߵ���ĿҪ����жϡ�
    HAL_UART_Transmit(&huart2,(uint8_t *)buff,i,0xffff);//�����������ݣ����ںţ����ݣ����������ʱ�䣩
    va_end(arg_ptr);
}

//����USART���ڵ��жϻص�����HAL_UART_RxCpltCallback��ͳһ����ڡ�USART1.C���ļ��С�

