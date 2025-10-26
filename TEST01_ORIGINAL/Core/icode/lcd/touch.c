/*
 * oled.c
 *
 *  Created on: Jun 11, 2022
 *      Author: Administrator
 */

/*
//�������ҳ�Ʒ
//����ϵ�п�����Ӧ�ó���
//��ע΢�Ź��ںţ����ҵ���
//���ҿ������������� www.DoYoung.net
//������ѿ����н�ѧ��Ƶ�����ؼ������ϣ�������������
//�������ݾ��� ����������ҳ www.doyoung.net
*/

#include "../../Core/icode/lcd/touch.h"

//�����������֧��GT9147��GT911����оƬ�����·��Ĵ�����ַ�������ݣ�

uint8_t TOUCH_ID[5]={0};//����оƬID�루TOUCH_Init��ʼ�����ʹ�ã���ASCII�뷽ʽ�ġ�9147����911����
uint16_t TOUCH_X[TOUCH_MAX];//��ǰ����ʱ��X����
uint16_t TOUCH_Y[TOUCH_MAX];//��ǰ����ʱ��Y����
uint8_t  TOUCH_STA;//��������ǰ����״̬

const uint16_t GT9xxx_TP_DAT[6]={GT9xxx_TP1,GT9xxx_TP2,GT9xxx_TP3,GT9xxx_TP4,GT9xxx_TP5};//5�������������I2C�ӵ�ַ������GT911��GT9xxx��ַ�д�λ��

const uint8_t GT9xxx_Config[]={//GT9xxx�������ݣ������üĴ���0x8047��ַ��ʼд�룬�����GT9147���ָ�ϡ���
0X60,0XE0,0X01,0X20,0X03,0X05,Module_Switch1,0X00,0X02,0X08,0X1E,0X08,0X50,0X3C,0X0F,0X05,0X00,0X00,0XFF,0X67,0X50,
0X00,0X00,0X18,0X1A,0X1E,0X14,0X89,0X28,0X0A,0X30,0X2E,0XBB,0X0A,0X03,0X00,0X00,0X02,0X33,0X1D,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X32,0X00,0X00,0X2A,0X1C,0X5A,0X94,0XC5,0X02,0X07,0X00,0X00,0X00,0XB5,0X1F,0X00,
0X90,0X28,0X00,0X77,0X32,0X00,0X62,0X3F,0X00,0X52,0X50,0X00,0X52,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0X0F,0X03,0X06,0X10,0X42,
0XF8,0X0F,0X14,0X00,0X00,0X00,0X00,0X1A,0X18,0X16,0X14,0X12,0X10,0X0E,0X0C,0X0A,0X08,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X29,0X28,0X24,0X22,0X20,
0X1F,0X1E,0X1D,0X0E,0X0C,0X0A,0X08,0X06,0X05,0X04,0X02,0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF
};

void GT9xxx_Write_Config(uint8_t save){//����GT9xxx_Config�������ݣ���ʼ��ʱʹ�ã���������1�������ݵ�GT9xxxоƬ��0�����棩
	uint8_t buf[2];
	uint8_t i=0;
	buf[0]=0;
	buf[1]=save;//�����Ƿ��������ݱ��浽GT9147/GT911оƬ�У�0�����棬1���棩
	for(i=0;i<sizeof(GT9xxx_Config);i++) buf[0]+=GT9xxx_Config[i];//����������ݵ�У���
    buf[0]=(~buf[0])+1;
	HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_COM,I2C_MEMADD_SIZE_16BIT,(uint8_t*)GT9xxx_Config,sizeof(GT9xxx_Config),1000);//������������
	HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_CHKSUM,I2C_MEMADD_SIZE_16BIT,buf,2,1000);//����У��ͽ��
}

uint8_t TOUCH_Init(void){//��������ʼ��������оƬGT9xxx��������1�ɹ���0ʧ�ܣ�
	uint8_t t;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(TOUTH_RST_GPIO_Port,TOUTH_RST_Pin, GPIO_PIN_RESET);//��RST��λ�ӿ����10����͵�ƽ����
	HAL_Delay(5);
	HAL_GPIO_WritePin(TOUTH_RST_GPIO_Port,TOUTH_RST_Pin, GPIO_PIN_SET);//
	HAL_Delay(10);
	//������ַ������ɺ󣬽�INT����Ϊ��������
	GPIO_InitStruct.Pin = TOUCH_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);//д��IO����
	HAL_Delay(100);
	TOUCH_STA = HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_PID,I2C_MEMADD_SIZE_16BIT,TOUCH_ID,4,1000);//������ƷID
	if(strcmp((char*)TOUCH_ID,"9147")==0 || strcmp((char*)TOUCH_ID,"911")==0){//�ж�ID��"9147"��911����ASCII�뷽ʽ��ID��
		t=0X02;
		HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_COM,I2C_MEMADD_SIZE_16BIT,&t,1,1000);//дָ����ƣ���λоƬ
		HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_Ver,I2C_MEMADD_SIZE_16BIT,&t,1,1000); //��ȡ�����ļ��汾��
		if(t<0X60)GT9xxx_Write_Config(1);//�жϰ汾�Ź�Сʱ//д�������ļ���������1�������ݵ�GT9xxxоƬ��0�����棩
		HAL_Delay(10);//��ʱ
		t=0;//GT9xxx_COM��ֵ0��������״̬
		HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_COM,I2C_MEMADD_SIZE_16BIT,&t,1,1000);//д��ָ����ƣ����������Ĺ���״̬
		return 1;//���سɹ�
	}
	return 0;//����ʧ��
}

void TOUCH_Read(uint8_t dir){//��ȡ������״̬���д�����ı�TOUCH_STA������5��������������TOUCH_X[]��TOUCH_Y[]�����У�
	//TOUCH_STA��BIT0-BIT3λ���津����������BIT4����״̬��1������0�ſ�����BIT7���ݸ���״̬��1�����Ѹ��£�0����δ׼���ã�
	//����������������ķ������touch.h�ļ���
	uint8_t buf[4];
	uint8_t i=0,STA=0;
	if(HAL_GPIO_ReadPin(GPIOG,TOUCH_INT_Pin)==GPIO_PIN_RESET){//��INT�ӿڵĵ�ƽ�����뽫Module_Switch1����Ϊ0x35�½��ش�����
		HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_STA,I2C_MEMADD_SIZE_16BIT,&STA,1,1000); //��ȡ������״̬�ܱ�־λ
		if((STA&0X80) && ((STA&0x0F)<=TOUCH_MAX)){//�ж�BIT7�ľ�����־λ�ǲ���1��ͬʱBIT0-3�Ĵ���������ҪС��������
			TOUCH_STA = STA;//���ܱ�־λ�е����ݷ���TOUCH_STAȫ�ֱ�־λ��������������ʹ��TOUCH_STA��
			for(i=0;i<5;i++){//д������֮ǰ����0
				TOUCH_X[i]=0;TOUCH_Y[i]=0;//�Ĵ�����0
			}
			for(i=0;i<(STA&0x0F);i++){//ѭ������5������ֵ
				HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_TP_DAT[i],I2C_MEMADD_SIZE_16BIT,buf,4,1000); //��ȡXY����ֵ
				if(dir==Portrait){//�ж��Ǻ�������������0��������1�Ǻ�����
					TOUCH_X[i]=((uint16_t)buf[1]<<8)+buf[0];//д������ֵ��ȫ��X�������飨��������д�룩
					TOUCH_Y[i]=((uint16_t)buf[3]<<8)+buf[2];//д������ֵ��ȫ��Y�������飨��������д�룩
				}else if(dir==Landscape){
					TOUCH_Y[i]=LCD_Width-(((uint16_t)buf[1]<<8)+buf[0]);//д������ֵ��ȫ��X�������飨��������д�룩
					TOUCH_X[i]=((uint16_t)buf[3]<<8)+buf[2];//д������ֵ��ȫ��Y�������飨��������д�룩
				}else if(dir==Portrait_reversal){
					TOUCH_X[i]=LCD_Width-(((uint16_t)buf[1]<<8)+buf[0]);//д������ֵ��ȫ��X�������飨��������д�룩
					TOUCH_Y[i]=LCD_Height-(((uint16_t)buf[3]<<8)+buf[2]);//д������ֵ��ȫ��Y�������飨��������д�룩
				}else if(dir==Landscape_reversal){
					TOUCH_Y[i]=((uint16_t)buf[1]<<8)+buf[0];//д������ֵ��ȫ��X�������飨��������д�룩
					TOUCH_X[i]=LCD_Height-(((uint16_t)buf[3]<<8)+buf[2]);//д������ֵ��ȫ��Y�������飨��������д�룩
				}
			}
			i=0;
			HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_STA,I2C_MEMADD_SIZE_16BIT,&i,1,1000);//���������־
		}
	}
}

/*********************************************************************************************
 * ���ҵ��� www.doyoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/
