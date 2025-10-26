

/*
//�������ҳ�Ʒ
//����ϵ�п�����Ӧ�ó���
//��ע΢�Ź��ںţ����ҵ���
//���ҿ������������� www.DoYoung.net/YT 
//������ѿ����н�ѧ��Ƶ�����ؼ������ϣ�������������
//�������ݾ��� ����������ҳ www.doyoung.net
*/

/*
���޸���־��
1-20170919 ������
2-20230916 ����3�ָ�ʽ���Զ���У���뺯��MY1690_CMD1~MY1690_CMD13

*/




#include "../../Core/icode/my1690/my1690.h"

//MY1690ͨ�ţ�ʹ��USART6��printf����
//���÷�����MY1690_printf("123"); //��USART6�����ַ�123
void MY1690_printf (char *fmt, ...)
{
    char buff[USART6_REC_LEN+1];  //���ڴ��ת��������� [����]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, USART6_REC_LEN+1,fmt,arg_ptr);//����ת��
    i=strlen(buff);//�ó����ݳ���
    if(strlen(buff)>USART6_REC_LEN)i=USART6_REC_LEN;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
    HAL_UART_Transmit(&huart6,(uint8_t *)buff,i,0xffff);//���ڷ��ͺ��������ںţ����ݣ����������ʱ�䣩
    va_end(arg_ptr);
}
//����USART���ڵ��жϻص�����HAL_UART_RxCpltCallback��ͳһ����ڡ�USART1.C���ļ��С�

void SPEAKER(uint8_t a)//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
{
	if(a)HAL_GPIO_WritePin(SPEAKER_GPIO_Port,SPEAKER_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(SPEAKER_GPIO_Port,SPEAKER_Pin,GPIO_PIN_SET);
}

uint8_t SD_READY(void)//SD��TF���������⹦�ܣ�����0Ϊ�޿���1Ϊ�п���
{
	if(HAL_GPIO_ReadPin(SD_READY_GPIO_Port,SD_READY_Pin)==0){//�ж϶˿ڵ�ƽ
		HAL_Delay(20);//��ʱ20ms�㿪��ι����еĶ���
		if(HAL_GPIO_ReadPin(SD_READY_GPIO_Port,SD_READY_Pin)==0)
			return 1;//���˿�Ϊ�͵�ƽʱ����1
	}return 0;//��δ��⵽�͵�ƽʱ����0
}

void MY1690_Init(void){ //��ʼ��
	//��MY1690�뵥Ƭ��USB�����豸�˿ڹ��ã���ʹ��MY169ǰ��Ҫ����Ƭ���˿�����Ϊ��������ģʽ����ֹ�໥���š�
	//������Ҫ��һ��������ͬʱʹ�����������ܣ��ɲ���Ҫ����GPIO��ֻҪ��MX����ɳ�ʼ���ü��ɡ�
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /*Configure GPIO pins : PC8 PC9 PC10 PC11 PC12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
	                          |GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA11 PA12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PD2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	HAL_UART_Receive_IT(&huart6,(uint8_t *)&USART6_NewData, 1); //��������6��MP3�������ж�
	USART6_RX_STA=0;//��־λ��0
	MY1690_STOP(); //�ϵ��ʼ������һ��ָ���MP3оƬ
	SPEAKER(0);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}
void MY1690_PLAY(void){ //����/��ͣ
	MY1690_printf("\x7e\x03\x1C\x1F\xef"); //���� \x ���ʮ����������
	SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}
void MY1690_PAUSE(void){ //��ͣ
	MY1690_printf("\x7e\x03\x12\x11\xef");
	SPEAKER(0);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}
void MY1690_PREV(void){ //��һ��
	MY1690_printf("\x7e\x03\x14\x17\xef");
	SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}
void MY1690_NEXT(void){ //��һ��
	MY1690_printf("\x7e\x03\x13\x10\xef");
	SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}
void MY1690_VUP(void){ //������1
	MY1690_printf("\x7e\x03\x15\x16\xef");
}
void MY1690_VDOWN(void){ //������1
	MY1690_printf("\x7e\x03\x16\x15\xef");
}
void MY1690_STOP(void){ //ֹͣ
	MY1690_printf("\x7e\x03\x1E\x1D\xef");
	SPEAKER(0);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}

void MY1690_CMD1(uint8_t a){ //�޲�����ָ���  a������
	uint8_t buf[5]={0x7e,3,a,3^a,0xef};//���巢�������õ����飨3^a��У����㣩
	HAL_UART_Transmit(&huart6,(uint8_t *)buf,5,0xffff);//���ڷ��ͺ���
	SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}
void MY1690_CMD2(uint8_t a,uint8_t b){ //�в�����ָ��� a������ b����
	uint8_t buf[6]={0x7e,4,a,b,4^a^b,0xef};
	HAL_UART_Transmit(&huart6,(uint8_t *)buf,6,0xffff);//���ڷ��ͺ���
	SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}
void MY1690_CMD3(uint8_t a,uint16_t b){ //�в�����ָ��� a������ b������16λ��
	uint8_t buf[7]={0x7e,5,a,b/0x100,b%0x100,5^a^b/0x100^b%0x100,0xef};
	HAL_UART_Transmit(&huart6,(uint8_t *)buf,7,0xffff);//���ڷ��ͺ���
	SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}


/*********************************************************************************************
 * �������� www.DoYoung.net
 * ���ҵ��� www.DoYoung.net/YT 
*********************************************************************************************/


/*
ѡ��IO�ӿڹ�����ʽ��
GPIO_Mode_AIN ģ������
GPIO_Mode_IN_FLOATING ��������
GPIO_Mode_IPD ��������
GPIO_Mode_IPU ��������
GPIO_Mode_Out_PP �������
GPIO_Mode_Out_OD ��©���
GPIO_Mode_AF_PP �����������
GPIO_Mode_AF_OD ���ÿ�©���
*/
