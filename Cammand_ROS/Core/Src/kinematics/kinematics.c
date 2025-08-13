/*
 * kinematics.c
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */

#include "kinematics.h"

// Calculate wheel velocities based on robot velocity and orientation
void CalculateKinematics(const Three_wheelsomniconfig* config, int16_t Vx, int16_t Vy, int16_t Vw, float Z_ang, WheelData* wheels) {
    double wheel_angles[3];

    // Calculate individual wheel angles based on base orientation and input Z_angle
    for (int i = 0; i < 3; i++) {
        wheel_angles[i] = config->theta[i] + (Z_ang * M_PI / 180.0);

        // Calculate wheel velocities using kinematic equations
        wheels->velocities[i] = (int16_t)(
            Vy * cos(wheel_angles[i]) +
            Vx * sin(wheel_angles[i]) +
            Vw * (config->Lx + config->Ly)

        );
    }
}
