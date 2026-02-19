#ifndef __LCD_H
#define __LCD_H		

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include "main.h"
#include "../delay/delay.h"
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"
 
//����������ֻ������NT35510/RM68120/OTM8009AоƬ����ʾ�ֱ���800x480��
#define LCD_ID_NT35510		0x00008000  //NT35510��IDֵ0x00008000
#define LCD_ID_RM68120		0x00008200  //RM68120��IDֵ0x00008200
#define LCD_ID_OTM8009		0x00004000  //OTM8009A��IDֵ0x00004000

#define LCD_PRINTF_LEN  	200 //����printf����һ�ε��ַ�����

#define LCD_Height			800 //����LCD����ģ�� �̶��ֱ���800x480
#define LCD_Width			480 //��������Ϊԭʼ��λ�����������¸߶�LCD_Height���̱������ҿ��LCD_Width��

#define LCD_COM				(uint32_t *)0X60000000  //��A18��ַ�˿���0��ĵ�ַ����ʾ��LCD����ָ�
#define LCD_DAT				(uint32_t *)0X60080000  //��A18��ַ�˿���1��ĵ�ַ����ʾ��LCD�������ݣ�
//������/ָ���л��˿�RS���ӵ�A18ʱ�������ַƫ����0x40000������Bank1ѡ����FSMC_NE1����ַ��0x60000000��
//�õ�����ʹRS��1�ĵ�ַ��0x60040000��������0110 0000 0000 0100 0000 0000 0000 0000��
//��������ִ��ʱ��ַƫ��ֵ������1λ������������ʱҪ����A19�˿ڣ����ƺ�������A18�������������õ�ַ��0X60080000

extern uint16_t SET_Height; //������Ļ�ֱ��ʸ߶�
extern uint16_t SET_Width; //������Ļ�ֱ��ʿ��
extern uint16_t SET_GRAM; //����GRAMͼ�μĴ���
extern uint16_t SET_X; //����X����
extern uint16_t SET_Y; //�豸Y����
extern uint32_t LCD_ID; //��ȡ�Ĳ���ID��żĴ���

extern SRAM_HandleTypeDef hsram1;    //FSMC���ߵ�SRAM������˴�����LCDģ��ͨ�ţ�

//�����뱳����ɫ���ã���ʹ��LCD_printf����֮ǰ���ã���:ForeColor=White;BackColor=Blue;��
extern uint16_t  ForeColor;//������ɫ�Ĵ�����ȫ���ַ�/ͼ�Σ�
extern uint16_t  BackColor;//������ɫ��ȫ�֣�

//ɨ�跽����
#define LRUD  0x00 		//������,���ϵ���
#define LRDU  0x80 		//������,���µ���
#define RLUD  0x40 		//���ҵ���,���ϵ���
#define RLDU  0xC0 		//���ҵ���,���µ���
#define UDLR  0x20 		//���ϵ���,������
#define UDRL  0x60 		//���ϵ���,���ҵ���
#define DULR  0xA0 		//���µ���,������
#define DURL  0xE0		//���µ���,���ҵ���

//���ò�ɫֵ
#define LCD_White       	0xFFFF//����
#define LCD_Black        	0x0000//����
#define LCD_Blue         	0x001F//��
#define LCD_Purple         	0XF81F//��
#define LCD_SkyBlue			0X07FF//����
#define LCD_Red          	0xF800//��
#define LCD_Magenta      	0xF81F//���
#define LCD_Green        	0x07E0//��
#define LCD_Cyan         	0x7FFF//��
#define LCD_Yellow       	0xFFE0//��
#define LCD_Brown 			0XBC40//����
#define LCD_Maroon	 		0XFC07//��ɫ
#define LCD_Gray  			0X8430//���

//�ײ���������
void LCD_Write_COM(uint16_t R);//��LCDдָ��COM
void LCD_Write_DAT(uint16_t D);//��LCDд����DAT
uint16_t LCD_Read_DAT(void);//��LCD��RAM�ռ�
void LCD_Write_REG(uint16_t R,uint16_t D);//дLCD�Ĵ�������������ַ�����ݣ�
uint16_t LCD_Read_REG(uint16_t R);//��LCD�Ĵ�����������LCD�Ĵ�����ַ�����أ��������ݣ�
void LCD_Write_Cursor(uint16_t x,uint16_t y);//���ù��λ�ã�������X���꣬Y���꣩
//LCD��������
uint8_t LCD_Init(void);//LCD������ʼ��������ֵ1����ʼ���ɹ���0����ʼ��ʧ�ܣ�
void LCD_Direction(uint8_t dir);//����LCD��ʾ����
void LCD_ON(void);//����ʾ�ͱ���
void LCD_OFF(void);//����ʾ�ͱ���
void LCD_CLEAR(uint16_t Color);//��������ɫ������
//ASCII�ַ���ʾ
void LCD_DISPLAY_ASCII(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);//��������ʾһ���ַ�
void LCD_printf_ASCII(uint8_t row,uint8_t column,uint8_t size,char *fmt, ...);//LCDר�õ�printf����
//������ʾ
void LCD_DISPLAY_CHS(uint16_t x,uint16_t y,uint16_t w,uint8_t size,uint8_t overlay);
void LCD_printf_CHS(uint16_t row,uint16_t column,uint8_t size,char *fmt, ...);
//ʸ��ͼ��ʾ
void LCD_Vector_Point(uint16_t x,uint16_t y);//���Ƶ����ص㣨������X���꣬Y���꣩
void LCD_Vector_Point2(uint16_t x,uint16_t y,uint16_t COLOR);//���Ƶ����ص�2��������X���꣬Y���꣬��ɫ��
void LCD_Vector_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);//����ʸ���߶Σ����������X�����Y���յ�X���յ�Y���꣩
void LCD_Vector_Circle(uint16_t x0,uint16_t y0,uint8_t r);//����ʸ����Բ�Σ�������Բ��X���꣬Բ��Y���꣬�뾶��
void LCD_Vector_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);//����ʸ�����ľ��Σ����������Ͻ�X�����Ͻ�Y�����½�X�����½�Y���꣩
void LCD_Vector_Rectangle_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);//����ʸ��ʵ�ľ��Σ����������Ͻ�X�����Ͻ�Y�����½�X�����½�Y����,��ɫ��
//BMPͼƬ��ʾ
void LCD_DISPLAY_BMP(uint16_t sx,uint16_t sy,uint16_t *COLOR);//����BMPλͼ�����������Ͻ�X�����Ͻ�Y����,BMPλͼ���飩
//�û��Զ����ͼƬ��ʾ
void LCD_DISPLAY_BMP_YoungTalkLogo(uint16_t sx,uint16_t sy);//��ʾ����LOGO���Զ���ͼƬ��ʾ���������������X�����Y���꣬��ͼƬ���Ͻǣ�

#endif  
	 
/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/
