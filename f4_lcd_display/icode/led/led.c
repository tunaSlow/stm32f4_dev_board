/*
 * led.c
 *
 *  Created on: Oct 22, 2021
 *      Author: Administrator
 */

#include "led.h"

void LED_1(uint8_t a)//LED1独立控制函数（0为熄灭，其他值为点亮）
{
	if(a)HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_SET);
}
void LED_2(uint8_t a)//LED2独立控制函数（0为熄灭，其他值为点亮）
{
	if(a)HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_SET);
}
void LED_3(uint8_t a)//LED3独立控制函数（0为熄灭，其他值为点亮）
{
	if(a)HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_SET);
}
void LED_4(uint8_t a)//LED4独立控制函数（0为熄灭，其他值为点亮）
{
	if(a)HAL_GPIO_WritePin(GPIOB,LED4_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOB,LED4_Pin,GPIO_PIN_SET);
}
void LED_ALL(uint8_t a)//LED1~4整组操作函数（低4位的1/0状态对应4个LED亮灭，最低位对应LED1）
{
	if(a&0x01)HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_SET);
	if(a&0x02)HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_SET);
	if(a&0x04)HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_SET);
	if(a&0x08)HAL_GPIO_WritePin(GPIOB,LED4_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOB,LED4_Pin,GPIO_PIN_SET);
}
void LED_1_Contrary(void){
	HAL_GPIO_WritePin(GPIOG,LED1_Pin,1-HAL_GPIO_ReadPin(GPIOG,LED1_Pin));
}
void LED_2_Contrary(void){
	HAL_GPIO_WritePin(GPIOG,LED2_Pin,1-HAL_GPIO_ReadPin(GPIOG,LED2_Pin));
}
void LED_3_Contrary(void){
	HAL_GPIO_WritePin(GPIOG,LED3_Pin,1-HAL_GPIO_ReadPin(GPIOG,LED3_Pin));
}
void LED_4_Contrary(void){
	HAL_GPIO_WritePin(GPIOB,LED4_Pin,1-HAL_GPIO_ReadPin(GPIOB,LED4_Pin));
}
