/*
 * Robotmovement.c
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */

#include "Robot_base_movement.h"
#include "../pid/pid.h"
#include "../kinematics/kinematics.h"
#include "../Bracket_track/Track.h"


Three_wheelsomniconfig Robot_dimension_param = {

		 .Lx = 0.36,              // meters
		 .Ly = 0.36,              // meters
		 .wheel_diameter = 0.152, // meters
		 .theta = {30.0 * M_PI/180.0, 150.0 * M_PI/180.0, -90.0 * M_PI/180.0}

	  };



WheelData Outputs_wheels_speed = {

		 .velocities = {0,0,0}

	    };



pid_t Con_mode;


extern int16_t  Vx;
extern int16_t  Vy;
extern int16_t  Vw;
extern float    Tracking_angle;
extern int16_t  Wheels_motor_speed[3];
extern int16_t  Set_speed_M[8];

void Robot_base_movement_init(void){

	PID_struct_init(&Con_mode ,POSITION_PID,12000,0,70.0,0,22);


}


void Robot_base_movement_start(Axis_speed_param * param,Equation_mode_param * mode_param){


	 Wheels_motor_speed[0]=Outputs_wheels_speed.velocities[0];
	 Wheels_motor_speed[1]=Outputs_wheels_speed.velocities[1];
	 Wheels_motor_speed[2]=Outputs_wheels_speed.velocities[2];


if(mode_param->equation_mode==0)
  CalculateKinematics(&Robot_dimension_param, param->VX, param->VY, param->VW, param->Z_ANGLE, &Outputs_wheels_speed);

if(mode_param->equation_mode==1)
  CalculateKinematics(&Robot_dimension_param, param->VX, param->VY, param->VW, 0, &Outputs_wheels_speed);

if(mode_param->equation_mode==2)
  CalculateKinematics(&Robot_dimension_param, param->VX, param->VY, pid_calc(&Con_mode ,( param->Z_ANGLE ), Vw/18), param->Z_ANGLE, &Outputs_wheels_speed);

if(mode_param->equation_mode==3){
	Wheels_motor_speed[0]=Vx;
    Wheels_motor_speed[1]=Vy;
    Wheels_motor_speed[2]=Vw;
}

}

