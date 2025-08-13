/*
 * Robotmovement.h
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */

#ifndef SRC_ROBOT_MOVEMENT_ROBOT_BASE_MOVEMENT_H_
#define SRC_ROBOT_MOVEMENT_ROBOT_BASE_MOVEMENT_H_

#include "stm32g4xx_hal.h"

typedef struct {
	int16_t VX;
	int16_t VY;
	int16_t VW;
	int16_t Z_ANGLE;



}Axis_speed_param ;



typedef struct {

	uint8_t equation_mode;

}Equation_mode_param;

void Robot_base_movement_init(void);
void Robot_base_movement_start(Axis_speed_param * param,Equation_mode_param * mode_param);

#endif /* SRC_ROBOT_MOVEMENT_ROBOT_BASE_MOVEMENT_H_ */
