/*
 * kinematics.h
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <math.h>
#include "stm32g4xx_hal.h"

// Structure to hold wheel velocities
typedef struct {
    int16_t velocities[3];  // Wheel velocities (m/s)
} WheelData;

// Robot physical constants
typedef struct {
    double Lx;              // Distance in x-direction (meters)
    double Ly;              // Distance in y-direction (meters)
    double wheel_diameter;  // Wheel diameter (meters)
    double theta[3];        // Fixed wheel orientation angles (radians)
} Three_wheelsomniconfig;

void CalculateKinematics(const Three_wheelsomniconfig* config, int16_t Vx, int16_t Vy, int16_t Vw, float Z_ang, WheelData* wheels);

#endif /* KINEMATICS_H_ */
