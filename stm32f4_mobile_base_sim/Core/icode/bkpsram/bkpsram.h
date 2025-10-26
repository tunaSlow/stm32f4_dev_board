/*
 * flash.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef ICODE_BKPSRAM_BKPSRAM_H_
#define ICODE_BKPSRAM_BKPSRAM_H_

#include "stm32f4xx_hal.h"

#define BKPSRAM_BEGIN 	0x40024000 //����SRAM����ʼ��ַ
#define BKPSRAM_END  	0x40024FFF //����SRAM�Ľ�����ַ

void US_BKPSRAM_INIT(void);//����SRAM��ʼ��
uint8_t US_BKPSRAM_WRITE_BYTE(uint32_t Bkpsram_Add,uint8_t *pBuffer,uint32_t Num); //8λ�ֽ�д����SRAM
void US_BKPSRAM_READ_BYTE (uint32_t Bkpsram_Add,uint8_t *pBuffer,uint32_t Num);    //8λ�ֽڶ�����SRAM

#endif /* ICODE_BKPSRAM_BKPSRAM_H_ */

/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/
