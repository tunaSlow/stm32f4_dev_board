/*
 * flash.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/flash/flash.h"

uint8_t STMFLASH_GetFlashSector(uint32_t addr){//�ò��������ĵ�ַ�ó������ڵ�������������32λ��ַ������ֵ�������ţ�
	if(addr<ADDR_FLASH_SECTOR_1) return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2) return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3) return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4) return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5) return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6) return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7) return FLASH_SECTOR_6;
	else if(addr<ADDR_FLASH_SECTOR_8) return FLASH_SECTOR_7;
	else if(addr<ADDR_FLASH_SECTOR_9) return FLASH_SECTOR_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;
	return FLASH_SECTOR_11;
}

uint8_t US_FLASH_WRITE_BYTE(uint32_t WriteAddr,uint8_t *pBuffer,uint32_t Num){//������������8λ�ֽ�д��FLASH����������ַ���������ݣ�������
	FLASH_EraseInitTypeDef User_Flash_Erase;//�����ṹ��
	uint32_t SectorError=0;
	static uint32_t addrx=0;
	uint32_t endaddr=0;
	//׼������
	if(WriteAddr < STM32_FLASH_BASE)return 1;//���ַ��ҪFLASH��ַ���䣬���ش������1
	HAL_FLASH_Unlock();//����FLASH
	if(FLASH_WaitForLastOperation(FLASH_WAITETIME)!=HAL_OK)return 2; //�ȴ���ɣ�ʧ�ܣ����ش�����2��
	addrx   = WriteAddr;//д�����ʼ��ַ
	endaddr = WriteAddr + Num;//д��Ľ�����ַ
	//��������
	if(addrx < STM32_FLASH_END){//�жϵ�ַ�Ƿ񳬳�FLASH����������Χ
		while(addrx < endaddr){//ѭ������д���ַ�ڲ���0xFFFFFFFF����������
			if(US_FLASH_READ_WORD(addrx) != 0XFFFFFFFF){//�жϵ�ǰ��ַ�����ǲ���0XFFFFFFFF���粻���������ַ��������
				User_Flash_Erase.TypeErase = FLASH_TYPEERASE_SECTORS;//�������ͣ���������
				User_Flash_Erase.Banks = FLASH_BANK_1; //FLASH���ڵ�Ƭѡ����STM32F407ֻ��FLASH_BANK_1��
				User_Flash_Erase.Sector = STMFLASH_GetFlashSector(addrx);//Ҫ����������
				User_Flash_Erase.NbSectors = 1;//һ��ֻ����һ������
				User_Flash_Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;//����FLASH��ѹ��Χ2.7~3.6V
				if(HAL_FLASHEx_Erase(&User_Flash_Erase,&SectorError) != HAL_OK)return 3;//��ʼ����������ʧ�ܣ����ش������3��
			}else addrx+=4;//�����ַ��4����ʹ��32λ�������Լ�4��
			if(FLASH_WaitForLastOperation(FLASH_WAITETIME)!=HAL_OK)return 4; //�ȴ���ɣ�ʧ�ܣ����ش������4��
		}
	}
	//д������
	if(FLASH_WaitForLastOperation(FLASH_WAITETIME) == HAL_OK){//�ȴ�FLASH׼������
		 while(WriteAddr < endaddr){//ѭ��д�룬ֱ�����һ����ַendaddr
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,WriteAddr,*pBuffer)!=HAL_OK)return 5;//д�����ݣ�ʧ�ܣ����ش������5��
			WriteAddr++;pBuffer++;//��ַ��1���������������1
		}
	}
	HAL_FLASH_Lock();//����FLASH
	return 0;
}

void US_FLASH_READ_BYTE (uint32_t User_Flash_Add,uint8_t *pBuffer,uint32_t Num){ //���ֽڶ���FLASH��������FLASH��ַ��������飬������
	uint32_t a=0;
	while(a < Num){//ѭ����ȡ��ֱ���������
    	*pBuffer = *(__IO uint8_t*)(User_Flash_Add + a); //����1���ֽ�����
    	pBuffer++; a++;//ָ���ַ��1��������1
    }
}

uint32_t US_FLASH_READ_WORD (uint32_t User_Flash_Add){ //���ֶ���FLASH���ݣ�������FLASH��ַ�����أ�32λ���ݣ�
    uint32_t FLASH_DATA;
    FLASH_DATA = *(__IO uint32_t*)(User_Flash_Add); //��4���ֽ�����
    return FLASH_DATA;
}
/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/
