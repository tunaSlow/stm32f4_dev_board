/*
 * can.h
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */

#ifndef SRC_CAN_CAN_H_
#define SRC_CAN_CAN_H_


#include "stm32g4xx_hal.h"

void CAN_INIT(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
void CAN_tran();

#endif /* SRC_CAN_CAN_H_ */
