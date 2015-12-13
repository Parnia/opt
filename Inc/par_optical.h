#ifndef  H_par_optical_H
#define H_par_optical_H

#include "stm32f4xx_hal.h"
#include "IMU.h"
#include "MPC.h"
#include "mpu6050.h"
#include "math.h"

#define focal_pix 290 //290

#define acc_noise    1
#define opti_noise_par   5//5

#define  fps         30 

#define acc_threshold        4

#define	F_CUT_Gyro1			   	 5.0f 
#define FILTER_Gyro_par          1/(2*PI*F_CUT_Gyro1)

#define	F_CUT_Opti			   	 6.0f 
#define FILTER_Opti_par          1/(2*PI*F_CUT_Opti)

typedef struct
{
	
	float delta_X,delta_Y;
	float delta_X_Last,delta_Y_Last;
	float Gyro_X,last_Gyro_X;
	float Gyro_Y,last_Gyro_Y;
	float Gyro_X_sum;
	float Gyro_Y_sum;
	float delta_X_correct;
	float delta_Y_correct;

	
	float x_correction;
	float y_correction;
	
	float delta_X_correct2;
	float delta_Y_correct2;
	
	float last_delta_X_correct;
	float last_delta_Y_correct;
	float real_vel_X;
	float real_vel_Y;
	int count;
	int data_ready;
	float Pos_X;
	float Pos_Y;
	float Pos_Z;
	
	float acc_X;
	float acc_Y;
	
}optical_par_struct;

extern optical_par_struct optical_par;

void do_optical_par(_MPC *MPC,MPU_SENSOR *MPU_sen,optical_par_struct *opti);
void optical_kalman_init();


#endif