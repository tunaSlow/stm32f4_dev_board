#ifndef __MY1690_H
#define __MY1690_H	 

#include "stm32f4xx_hal.h" //HAL库文件声明
#include "main.h"
#include "../usart/usart.h"
#include <string.h>//用于字符串处理的库
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"


void MY1690_printf (char *fmt, ...);//使用USART6的printf函数
void SPEAKER(uint8_t a);//LM4871芯片使能控制函数（0为关闭，其他值为开启）
uint8_t SD_READY(void);//SD（TF）卡插入检测功能（返回0为无卡，1为有卡）
void MY1690_Init(void);//初始化

//直接输入的指令
void MY1690_PLAY(void);
void MY1690_PREV(void);//
void MY1690_NEXT(void);
void MY1690_PAUSE(void);
void MY1690_VUP(void);
void MY1690_VDOWN(void);
void MY1690_STOP(void);

void MY1690_CMD1(uint8_t a);//全部指令输入1
void MY1690_CMD2(uint8_t a,uint8_t b);//全部指令输入2
void MY1690_CMD3(uint8_t a,uint16_t b);//全部指令输入3
		 				    
#endif
