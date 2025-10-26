/*
 * relay.h
 *
 *  Created on: 2021年10月20日
 *      Author: Administrator
 */

#ifndef INC_RELAY_H_
#define INC_RELAY_H_

//继电器接口定义与初始化函数在MX中设置并生成在main.c文件中
#include "stm32f4xx_hal.h" //HAL库文件声明
#include "main.h" //IO定义与初始化函数在main.c文件

void RELAY_1(uint8_t c);//继电器控制1
void RELAY_2(uint8_t c);//继电器控制2

#endif /* INC_RELAY_H_ */

