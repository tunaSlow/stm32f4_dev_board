/*
 * uart.h
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */

#ifndef SRC_UART_UART_H_
#define SRC_UART_UART_H_

#include "stm32g4xx_hal.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void UART_Init(void);
#endif /* SRC_UART_UART_H_ */
