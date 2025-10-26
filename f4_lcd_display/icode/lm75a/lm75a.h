#ifndef __LM75A_H
#define __LM75A_H	 

#include "stm32f4xx_hal.h" //HAL库文件声明
#include "main.h" //IO定义与初始化函数在main.c文件中，必须引用

extern I2C_HandleTypeDef hi2c2;//声明I2C2的HAL库结构体

#define LM75A_ADD		0x9E	//器件地址
#define TEMP			0x00  	//温度子地址
#define CONF 			0x01  	//配置子地址

void LM75A_GetTemp(uint8_t *Tempbuffer);//读温度数据
void LM75_CONFIG(uint8_t cfg);//写配置寄存器
		 				    
#endif


/*********************************************************************************************
 * 洋桃电子 www.DoYoung.net
 * 部分程序代码复制自网络开源资料 如有侵权请联系我们处理
 * 洋桃电子原创程序代码部分均未声明版权 可自由复制使用 我们不对代码做任何担保
*********************************************************************************************/
