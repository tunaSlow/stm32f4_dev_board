/*
 * led.c
 *
 *  Created on: Oct 22, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/led/led.h"

void LED_1(uint8_t a)//LED1�������ƺ�����0ΪϨ������ֵΪ������
{
	if(a)HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED1_Pin,GPIO_PIN_SET);
}
void LED_2(uint8_t a)//LED2�������ƺ�����0ΪϨ������ֵΪ������
{
	if(a)HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED2_Pin,GPIO_PIN_SET);
}
void LED_3(uint8_t a)//LED3�������ƺ�����0ΪϨ������ֵΪ������
{
	if(a)HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOG,LED3_Pin,GPIO_PIN_SET);
}
void LED_4(uint8_t a)//LED4�������ƺ�����0ΪϨ������ֵΪ������
{
	if(a)HAL_GPIO_WritePin(GPIOB,LED4_Pin,GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOB,LED4_Pin,GPIO_PIN_SET);
}
void LED_ALL(uint8_t a)//LED1~4���������������4λ��1/0״̬��Ӧ4��LED�������λ��ӦLED1��
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
