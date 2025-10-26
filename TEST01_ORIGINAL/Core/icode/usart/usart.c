/*
 * usart1.c
 *
 *  Created on: Oct 20, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/usart/usart.h"

uint8_t USART1_RX_BUF[USART1_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.
uint16_t USART1_RX_STA=0;//����״̬���//bit15��������ɱ�־��bit14�����յ�0x0d��bit13~0�����յ�����Ч�ֽ���Ŀ
uint8_t USART1_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

uint8_t USART2_RX_BUF[USART2_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.
uint16_t USART2_RX_STA=0;//����״̬���//bit15��������ɱ�־��bit14�����յ�0x0d��bit13~0�����յ�����Ч�ֽ���Ŀ
uint8_t USART2_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

uint8_t USART3_RX_BUF[USART3_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.
uint16_t USART3_RX_STA=0;//����״̬���//bit15��������ɱ�־��bit14�����յ�0x0d��bit13~0�����յ�����Ч�ֽ���Ŀ
uint8_t USART3_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

uint8_t UART4_RX_BUF[UART4_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.
uint16_t UART4_RX_STA=0;//����״̬���//bit15��������ɱ�־��bit14�����յ�0x0d��bit13~0�����յ�����Ч�ֽ���Ŀ
uint8_t UART4_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

uint8_t USART6_RX_BUF[USART6_REC_LEN];//���ջ���,���USART_REC_LEN���ֽ�.
uint16_t USART6_RX_STA=0;//����״̬���//bit15��������ɱ�־��bit14�����յ�0x0d��bit13~0�����յ�����Ч�ֽ���Ŀ
uint8_t USART6_NewData;//��ǰ�����жϽ��յ�1���ֽ����ݵĻ���

//USB����ͨ�ţ�ʹ��USART1��printf����
//���÷�����USART1_printf("123"); //��USART1�����ַ�123
void USART1_printf (char *fmt, ...)
{
    char buff[USART1_REC_LEN+1];  //���ڴ��ת��������� [����]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, USART1_REC_LEN+1, fmt,  arg_ptr);//����ת��
    i=strlen(buff);//�ó����ݳ���
    if(strlen(buff)>USART1_REC_LEN)i=USART1_REC_LEN;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
    HAL_UART_Transmit(&huart1,(uint8_t  *)buff,i,0xffff);//���ڷ��ͺ��������ںţ����ݣ����������ʱ�䣩
    va_end(arg_ptr);
}

void  HAL_UART_RxCpltCallback(UART_HandleTypeDef  *huart)//�����жϻص�����
{
	if(huart ==&huart1)//�ж��ж���Դ������1��USBת���ڣ�
    {
		USART1_printf("%c",USART1_NewData); //���յ��������� a���ű��� ���ͻص���
       if((USART1_RX_STA&0x8000)==0){//����δ���
           if(USART1_RX_STA&0x4000){//���յ���0x0d
               if(USART1_NewData!=0x0a)USART1_RX_STA=0;//���մ���,���¿�ʼ
               else USART1_RX_STA|=0x8000;   //���������
           }else{ //��û�յ�0X0D
               if(USART1_NewData==0x0d)USART1_RX_STA|=0x4000;
               else{
                  USART1_RX_BUF[USART1_RX_STA&0X3FFF]=USART1_NewData; //���յ������ݷ�������
                  USART1_RX_STA++;  //���ݳ��ȼ�����1
                  if(USART1_RX_STA>(USART1_REC_LEN-1))USART1_RX_STA=0;//�������ݴ���,���¿�ʼ����
               }
           }
       }
       HAL_UART_Receive_IT(&huart1,(uint8_t *)&USART1_NewData,1); //���������ж�
    }
	if(huart ==&huart2)//�ж��ж���Դ������2��WIFIģ�飩
	{
		if(USART2_RX_STA<USART2_REC_LEN)//�����Խ�������
		{
			__HAL_TIM_SET_COUNTER(&htim2,0); //���������
			if(USART2_RX_STA==0) //ʹ�ܶ�ʱ��2���ж�
			{
				__HAL_TIM_ENABLE(&htim2); //ʹ�ܶ�ʱ��2
			}
			USART2_RX_BUF[USART2_RX_STA++] = USART2_NewData;//���½������ݷ�������
		}else{
			USART2_RX_STA|=0x8000;//ǿ�Ʊ�ǽ������
		}
		HAL_UART_Receive_IT(&huart2,(uint8_t *)&USART2_NewData,1); //�ٿ�������3�����ж�
	}
    if(huart ==&huart3)//�ж��ж���Դ������ģ�飩
    {
		if((USART3_RX_STA&0x8000)==0){//����δ��ɣ���USART3_RX_STA���λ1λ�涨�ǽ�����ɱ�־λ��
		   if(USART3_NewData==0x0A)//���յ�0x0A��ʾ���յ�������������ģ��ظ�������0x0AΪ��������
		   {
			   USART3_RX_STA|=0x8000;//�յ�0x0A���������
		   }
		   else{ //��û���յ�0x0A����������������ݲ���������1
			  USART3_RX_BUF[USART3_RX_STA&0X7FFF]=USART3_NewData; //���յ������ݷ�������
			  USART3_RX_STA++;  //���ݳ��ȼ�����1
			  if(USART3_RX_STA>(USART3_REC_LEN-1))USART3_RX_STA=0;//�������ݴ���,���¿�ʼ����
		   }
		}
		HAL_UART_Receive_IT(&huart3,(uint8_t *)&USART3_NewData,1); //�ٿ��������ж�
    }
    if(huart ==&huart4)//�ж��ж���Դ��RS485��
    {
    	UART4_RX_BUF[0]=UART4_NewData;//�����յ������ݷ��뻺�����飨��ֻ�õ�1�����ݣ�����ֻ���������[0]λ�ã�
    	UART4_RX_STA++;//���ݽ��ձ�־λ��1
    	HAL_UART_Receive_IT(&huart4,(uint8_t *)&UART4_NewData, 1); //���������ж�
    }
}
