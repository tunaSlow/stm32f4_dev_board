/*
 * flash.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef ICODE_FLASH_FLASH_H_
#define ICODE_FLASH_FLASH_H_

#include "stm32f4xx_hal.h"

#define STM32_FLASH_BASE 0x08000000 //FLASH����ʼ��ַ
#define STM32_FLASH_END  0X1FFF0000 //FLASH�Ľ�����ַ
#define FLASH_WAITETIME 50000 //FLASH�ȴ���ʱʱ��

//FLASHģ�鹹��������
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000) // ����0, 16 Kbytes
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000) // ����1, 16 Kbytes
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000) // ����2, 16 Kbytes
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000) // ����3, 16 Kbytes
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000) // ����4, 64 Kbytes
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000) // ����5, 128 Kbytes
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000) // ����6, 128 Kbytes
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000) // ����7, 128 Kbytes
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000) // ����8, 128 Kbytes
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000) // ����9, 128 Kbytes
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) // ����10, 128 Kbytes
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) // ����11, 128 Kbytes

uint8_t US_FLASH_WRITE_BYTE(uint32_t WriteAddr,uint8_t *pBuffer,uint32_t Num);//8λ�ֽ�дFLASH
void US_FLASH_READ_BYTE (uint32_t User_Flash_Add,uint8_t *pBuffer,uint32_t Num);//8λ�ֽڶ�FLASH
uint32_t US_FLASH_READ_WORD (uint32_t User_Flash_Add);//32λ��FLASH

#endif /* ICODE_FLASH_FLASH_H_ */

/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ������Ӧ��������IoT�����壬�ɵ����ҵ��ӹ����ۿ���ѧ��Ƶ��������������
*********************************************************************************************/
