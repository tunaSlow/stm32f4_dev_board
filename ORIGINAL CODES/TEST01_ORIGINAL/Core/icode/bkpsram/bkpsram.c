/*
 * flash.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/bkpsram/bkpsram.h"

void US_BKPSRAM_INIT(void){//����SRAM��ʼ��
	__HAL_RCC_PWR_CLK_ENABLE();//ʹ��PWR��Դʱ�ӣ�������������ʱ�ӣ�
	HAL_PWR_EnableBkUpAccess(); //ʹ�ܱ�����д��ȡ��д������
	__HAL_RCC_BKPSRAM_CLK_ENABLE();//ʹ�ܱ���SRAMʱ��
	HAL_PWREx_EnableBkUpReg(); //ʹ�ܱ�������ѹ��
}

uint8_t US_BKPSRAM_WRITE_BYTE(uint32_t Bkpsram_Add,uint8_t *pBuffer,uint32_t Num){//��8λ�ֽ�д����SRAM����������ַ���������ݣ�������
	uint32_t a=0;
	if(Bkpsram_Add<BKPSRAM_BEGIN || (Bkpsram_Add+Num)>BKPSRAM_END)return 1;//���ַ���ڱ���SRAM��ַ���䣬���ش������1
	while(a < Num){//ѭ��д�룬ֱ���������
		*(__IO uint8_t *)(Bkpsram_Add+a) = *pBuffer;//����SRAMд������
		pBuffer++; a++;//ָ���ַ��1��������1
	}
	return 0;
}

void US_BKPSRAM_READ_BYTE (uint32_t Bkpsram_Add,uint8_t *pBuffer,uint32_t Num){ //���ֽڶ�������SRAM������������SRAM��ַ��������飬������
	uint32_t a=0;
	while(a < Num){//ѭ����ȡ��ֱ���������
    	*pBuffer = *(__IO uint8_t*)(Bkpsram_Add + a); //����1���ֽ�����
    	pBuffer++; a++;//ָ���ַ��1��������1
    }
}

/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ������Ӧ��������IoT�����壬�ɵ����ҵ��ӹ����ۿ���ѧ��Ƶ��������������
*********************************************************************************************/
