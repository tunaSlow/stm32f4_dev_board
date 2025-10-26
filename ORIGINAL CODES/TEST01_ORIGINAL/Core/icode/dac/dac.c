/*
 * dac.c
 *
 *  Created on: Dec 14, 2022
 *      Author: Administrator
 */


#include "../../Core/icode/dac/dac.h"

#include "../../Core/icode/dac/wav1.h"

#define wav1_Channel	DAC_CHANNEL_2	//wav1��Ƶ�����DACͨ�����ã���ѡDAC_CHANNEL_1��DAC_CHANNEL_2��
#define wav1_LEN		20324	//wav1��Ƶ��������������ã�����wav1.h�ļ��е����������޸ģ�
#define wav1_SPEED		125		//wav1��Ƶ�����ٶȣ������ٶ���125��
#define wav1_DATA		wav1	//��Ƶ���������
#define wav1_BIT		DAC_ALIGN_12B_R	//����λ����DAC_ALIGN_12B_R��12λ�Ҷ��룩
//��ע�⡿ֻ����������8λ��Ƶʱ��������������8λ��ֵ*16��=12λ���ֵ������*1~16�����ɿ���1~16��������

void DAC_PLAY_WAV1(uint8_t Vol){//DACֱ�Ӳ���ģ����Ƶ��������������1~16��0�Ǿ�����
	uint32_t a;
	DAC_Init1();//DAC��ʼ��1������ͨ��2����Ƶ�������
	DAC_SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
	HAL_DAC_Start(&hdac, wav1_Channel);//����DAC����
	HAL_Delay(100);//��ʱ
	if(Vol>16)Vol=16;//���������ޣ�������Ϊ�������
	for(a=0;a<wav1_LEN;a++){ //��Ƶ�������ݳ��ȣ�20029��
		HAL_DAC_SetValue(&hdac,wav1_Channel,wav1_BIT,(wav1_DATA[a]*Vol));//����DAC����
		delay_us(wav1_SPEED); //��ʱ
	}
	HAL_Delay(100);//��ʱ
	DAC_SPEAKER(0);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
	HAL_DAC_Stop(&hdac, DAC_CHANNEL_2);//ֹͣDAC����
}

void DAC_Init1(void){//DAC��ʼ��1������ͨ��2����Ƶ�������
  DAC_ChannelConfTypeDef sConfig = {0};
  /** DAC Initialization*/
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)Error_Handler();
  /** DAC channel OUT1 config*/
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)Error_Handler();
  /** DAC channel OUT2 config*/
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)Error_Handler();
}

void DAC_Init2(void){//DAC��ʼ��2������ͨ��2�����ǲ�������ã���ҪTIM2��MX��Ԥ�����úã�
  DAC_ChannelConfTypeDef sConfig = {0};
  /** DAC Initialization*/
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)Error_Handler();
  /** DAC channel OUT1 config*/
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)Error_Handler();
  /** DAC channel OUT2 config*/
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)Error_Handler();
  /** Configure Triangle wave generation on DAC OUT2*/
  if (HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_2, DAC_TRIANGLEAMPLITUDE_4095) != HAL_OK)Error_Handler();
}

void DAC_SPEAKER(uint8_t a){//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
	if(a)HAL_GPIO_WritePin(SPEAKER_GPIO_Port,SPEAKER_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(SPEAKER_GPIO_Port,SPEAKER_Pin,GPIO_PIN_SET);
}

//�ڵ������ǲ����ɺ���֮ǰ��������MX�п���TIM2��DAC������ȷ������ز�������
void DAC_Triangle(void){//DAC���ǲ�����
	DAC_Init2();//DAC��ʼ��2������ͨ��2�����ǲ�������ã���ҪTIM2��MX��Ԥ�����úã�
	HAL_TIM_Base_Start(&htim4);//������ʱ��4���������ǲ�����Ķ�ʱ��
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);//����DAC���ܣ�DAC OUT2��������ǲ���
	DAC_SPEAKER(1);//��Ƶ�Ŵ�оƬLM4871оƬʹ�ܿ��ƺ�����0Ϊ�رգ�����ֵΪ������
}


