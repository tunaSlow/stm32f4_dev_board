/*
 * rs485.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/rs485/rs485.h"

#include "main.h"

#include "../../Core/icode/usart/usart.h"
/*
RS485����ͨ�ţ�ʹ��UART4������RS485ר�õ�printf����
���÷�����RS485_printf("123"); //��UART4�����ַ�123
*/
void RS485_printf (char *fmt, ...)
{
    char buff[UART4_REC_LEN+1];  //���ڴ��ת��������� [����]
    uint16_t i=0;
    va_list arg_ptr;
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port,RS485_RE_Pin, GPIO_PIN_SET);//RS485�շ�ѡ����REΪ�ߵ�ƽ�����ͣ�
    va_start(arg_ptr,fmt);
    vsnprintf(buff, UART4_REC_LEN+1,fmt,arg_ptr);//����ת��
    i=strlen(buff);//�ó����ݳ���
    if(strlen(buff)>UART4_REC_LEN)i=UART4_REC_LEN;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
    HAL_UART_Transmit(&huart4,(uint8_t *)buff,i,0xffff);//���ڷ��ͺ��������ںţ����ݣ����������ʱ�䣩
    va_end(arg_ptr);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port,RS485_RE_Pin, GPIO_PIN_RESET);//RS485�շ�ѡ����REΪ�͵�ƽ�����գ�
}
//����USART���ڵ��жϻص�����HAL_UART_RxCpltCallback��ͳһ����ڡ�USART1.C���ļ��С�
