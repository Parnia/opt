#include "par_optical.h"

optical_par_struct optical_par;
_Kalman1x1 x_vel,y_vel;

void do_optical_par(_MPC *MPC,MPU_SENSOR *MPU_sen,optical_par_struct *opti)
{
	
	opti->acc_X = MPU_sen->acc_y - Mahony.pure_acc_y;  // dynamic_acc=acc - static_acc // sehtabe mehvara ja be ja??
	opti->acc_X = -100*((opti->acc_X*9.81f)/MPU_sen->Gravity); // scaling
	if(fabs(opti->acc_X) < acc_threshold)
		opti->acc_X = 0;	
	
	opti->acc_Y = MPU_sen->acc_x - Mahony.pure_acc_x;
	opti->acc_Y = -100*((opti->acc_Y*9.81f)/MPU_sen->Gravity);
	if(fabs(opti->acc_Y) < acc_threshold)
		opti->acc_Y = 0;
	
	
	opti->Gyro_X_sum += (MPU_sen->gyro_x);
	opti->Gyro_Y_sum += (MPU_sen->gyro_y);
	
	if (MPC->ready)
	{
		opti->delta_X = ((float)MPC->data[0])/(10);
		opti->delta_Y = ((float)MPC->data[1])/(10);
		
		opti->Gyro_X = (opti->Gyro_X_sum); //*DT ro bordam jaye dg
		opti->Gyro_Y = (opti->Gyro_Y_sum);
		
		opti->Gyro_X = opti->last_Gyro_X +((0.033)/(FILTER_Gyro_par + (0.033)))*(opti->Gyro_X - opti->last_Gyro_X);  
		opti->Gyro_Y = opti->last_Gyro_Y +((0.033)/(FILTER_Gyro_par + (0.033)))*(opti->Gyro_Y - opti->last_Gyro_Y);
		opti->last_Gyro_X = opti->Gyro_X;
		opti->last_Gyro_Y = opti->Gyro_Y;
		
		opti->x_correction=-tan(opti->Gyro_X * DT) * (focal_pix); //Dt=0.004
		opti->y_correction=tan(opti->Gyro_Y * DT) * (focal_pix);
		
//		opti->x_correction=-tan(opti->Gyro_X /fps) * focal_pix;
//		opti->y_correction=tan(opti->Gyro_Y /fps) * focal_pix;
		
		opti->delta_X_correct= opti->delta_X - opti->x_correction;
		opti->delta_Y_correct= opti->delta_Y - opti->y_correction;
		
//		opti->delta_X_correct2= opti->delta_X + tan(opti->Gyro_X * 0.004) * focal_pix;
//		opti->delta_Y_correct2= opti->delta_Y + tan(opti->Gyro_Y * 0.004) * focal_pix;
		
		opti->delta_X_correct = opti->last_delta_X_correct+((0.033)/(FILTER_Opti_par + (0.033)))*(opti->delta_X_correct - opti->last_delta_X_correct);  
		opti->delta_Y_correct = opti->last_delta_Y_correct+((0.033)/(FILTER_Opti_par + (0.033)))*(opti->delta_Y_correct - opti->last_delta_Y_correct);  
		opti->last_delta_X_correct = opti->delta_X_correct;
		opti->last_delta_Y_correct = opti->delta_Y_correct;
		
		opti->real_vel_X=  (opti->delta_X_correct * Ultra.point * fps) / focal_pix;  // *fps to change delta x to velcocity
		opti->real_vel_Y=  (opti->delta_Y_correct * Ultra.point * fps) / focal_pix;
		
		opti->data_ready=1;
		opti->Gyro_X_sum=0;
		opti->Gyro_Y_sum=0;		
		MPC->ready=0;
	}
	
	if(opti->data_ready == 1)
	{
		opti->data_ready=0;
		
		
		Update_Kalman1x1(&x_vel,opti->real_vel_X,opti->acc_X,opti_noise_par,acc_noise);	
		Update_Kalman1x1(&y_vel,opti->real_vel_Y,opti->acc_Y,opti_noise_par,acc_noise);	
		
		
	}
	
	else  
	{
		x_vel.state = x_vel.state + opti->acc_X * DT;
		y_vel.state = y_vel.state + opti->acc_Y * DT;
	}
	
}

void optical_kalman_init()
{
	
	x_vel.state=0;
	x_vel.C=1;
	x_vel.A=1;
	x_vel.B=DT;
	x_vel.P=1;
	
	y_vel.state=0;
	y_vel.C=1;
	y_vel.A=1;
	y_vel.B=DT;
	y_vel.P=1;

}
