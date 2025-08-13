/*
 * Track.c
 *
 *  Created on: May 11, 2025
 *      Author: Vannak
 */


#include "math.h"


extern float Tracking_angle;
extern float Z_angle , X_angle , Y_angle , Pos_X, Pos_Y , W_z ;


int TRACKING(){

return 	Tracking_angle = atan((3640+Pos_X)/(13630+Pos_Y))*(180/M_PI);

}



