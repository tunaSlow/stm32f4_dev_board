/*
 * uart.c
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */
#include "uart.h"

extern UART_HandleTypeDef huart1;

extern uint8_t Data_uart_1[1];
extern int16_t data_com[4];
extern int16_t Vx, Vy, Vw, Vl ,Vs;

extern float Z_angle, X_angle, Y_angle, Pos_X, Pos_Y, W_z;


void UART_Init(void){

	HAL_UART_Receive_IT(&huart1, Data_uart_1, 1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//Odometry
  if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, Data_uart_1, 1);

        static union {
            uint8_t DT[24];
            float Actual_val[6];
        } posture;

        static uint8_t count = 0;
        static uint8_t i = 0;
        switch (count) {
            case 0:
                if (Data_uart_1[0] == 0x0D) {
                    count++;
                } else {
                    count = 1;
                }
                break;
            case 1:
                if (Data_uart_1[0] == 0x0A) {
                    i = 0;
                    count++;
                } else {
                    count = 0;
                }
                break;
            case 2:
                posture.DT[i] = Data_uart_1[0];
                i++;
                if (i >= 24) {
                    i = 0;
                    count++;
                }
                break;
            case 3:
                if (Data_uart_1[0] == 0x0A) {
                    count++;
                } else {
                    count = 0;
                }
                break;
            case 4:
                if (Data_uart_1[0] == 0x0D) {

                    Z_angle = posture.Actual_val[0];
                    X_angle = posture.Actual_val[1];
                    Y_angle = posture.Actual_val[2];
                    Pos_X = posture.Actual_val[3];
                    Pos_Y = posture.Actual_val[4];
                    W_z = posture.Actual_val[5];

                    if (Pos_X != 0 && Pos_Y != 0 && Z_angle != 0) {
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                    }

                    if (Pos_X == 0 && Pos_Y == 0 && Z_angle == 0) {
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                    }

                }
                count = 0;
                break;
            default:
                count = 0;
                break;
        }
    }

}






