#include "Altitude.h"
#include "IMU.h"
#include "Odometery.h"
#include "Kalman.h"

_Ultra Ultra;
float vel_z;
int start_vel_z_kalman =0;
int counter_loop=0;



void Read_Srf(TIM_HandleTypeDef _htim,_Ultra* ultra)
{
	uint16_t count=0;
	ultra->fail = ultra->fail + 1;
	
	if(ultra->State == 0)
	{	
		if(ultra->ready == 1)
		{
			if(ultra->Begine < ultra->End)
				ultra->real = (((float)(ultra->End - ultra->Begine) /200)* 3.3f); //200;
			else
				ultra->real = (((float)(ultra->End - ultra->Begine + 0xffff) /200)* 3.3f);
			
			if(ultra->real < Max_Ultra_Thr)
			{
				ultra->ready = 2;
				
				if(ultra->End > ultra->last_Time)
					ultra->Diff_Time = ((float)(ultra->End - ultra->last_Time)/1000)/1000;
				else
					ultra->Diff_Time = ((float)(ultra->End - ultra->last_Time + 0xffff )/1000)/1000;
				
				ultra->last_Time = ultra->End;	
				
				ultra_filter_lpf(ultra); 	//** For Control Z Position Enable this		
				//Ultra.last_real=Ultra.real;		// DAME MOSABEGHATI GIR NADID DIGE!!!!
			}
			else
				ultra->ready = 0;			
		}	
		
		Ultra_Trig_ON;
		count = _htim.Instance->CNT ;	
		ultra->Begine = _htim.Instance->CNT ;	
		while(fabs((uint16_t)_htim.Instance->CNT - count)< 30);
		Ultra_Trig_OFF;
		ultra->State = 1;	
		ultra->fail = 0;		
	}	
	if(counter>200 | start_vel_z_kalman == 1)
	{
		
		Vel_z(&z_vel,ultra,0,_htim);			//** For Control Z Position Disable this

		start_vel_z_kalman=1;
	}
	if(ultra->State == 3)
		ultra->State = 0;
	
	if(ultra->fail >16)
		ultra->State = 0;
}

void Ultra_Kalman_init(void)
{
	
	Ultra_Z_Kalman.STATE[0][0] = 0;
	Ultra_Z_Kalman.STATE[1][0] = 0;
	
	Ultra_Z_Kalman.P[0][0] = 0;
	Ultra_Z_Kalman.P[0][1] = 0;
	Ultra_Z_Kalman.P[1][0] = 0;
	Ultra_Z_Kalman.P[1][1] = 0;

	
	Ultra_Z_Kalman.A[0][0] = 1;
	Ultra_Z_Kalman.A[0][1] = DT;
	Ultra_Z_Kalman.A[1][0] = 0;
	Ultra_Z_Kalman.A[1][1] = 1;
	
	Ultra_Z_Kalman.B[0][0] = 0.5f*DT*DT;
	Ultra_Z_Kalman.B[1][0] = DT;
	
	Ultra_Z_Kalman.C[0][0] = 1;
	Ultra_Z_Kalman.C[0][1] = 0;
}

void  ultra_filter_lpf(_Ultra* ultra)
{
	float filter,f_cut;
  // (6,100) (0.1,250)  
	//ultra->lpf_Gain =ultra->vel;
	ultra->lpf_Gain  = (ultra->real - ultra->last_point)/ultra->Diff_Time;

	if(fabs(ultra->lpf_Gain) < 500)
			f_cut=6;
	else
	{
			f_cut = (800 - fabs(ultra->lpf_Gain))/84;
			if(f_cut < 0.1f)
				f_cut = 0.1f;
	}
	
	filter =1/(2*3.14f*f_cut);			
	ultra->point = ultra->last_point +(ultra->Diff_Time/(filter + ultra->Diff_Time))*(ultra->real-ultra->last_point);  
	
	ultra->last_diff = ultra->diff;				
	ultra->diff =(float)(ultra->point - ultra->last_point)/ultra->Diff_Time; 
	ultra->last_point= ultra->point;
}

float Vel_z(_Kalman1x1 *Kalman_state,_Ultra *ultra,float acc,TIM_HandleTypeDef _htim)
{

		
	float ultra_noise=0;
	float acc_z=Mahony.Earth_acc_z;
	
	z_vel.last_state=z_vel.state;

	if(ultra->ready == 2)
	{
		ultra->ready = 0;	
  //	ultra->K_point = ultra->K_last_point +(ultra->Diff_Time/(FILTER_Ultra + ultra->Diff_Time))*(ultra->real-ultra->K_last_point);  
		
		ultra->K_point=ultra->real;
		ultra->vel = ((float)((ultra->K_point) - (ultra->K_last_point))/ultra->Diff_Time);
		ultra->main_vel=ultra->vel;
		ultra->vel= ultra->last_vel + ( ultra->Diff_Time /(ultra->Diff_Time + FILTER_VEL_lpf )*(ultra->vel - ultra->last_vel));
	  ultra->vel_hpf=(FILTER_VEL_hpf/(FILTER_VEL_hpf + ultra->Diff_Time) )*(ultra->vel - ultra->last_vel + ultra->last_vel_hpf) ;

			ultra->last_vel = ultra->vel;	
			ultra->K_last_point= ultra->K_point;	
	  ultra->last_vel_hpf = ultra->vel_hpf ;
//		if(counter_loop==0){
//			
//											if(fabs( ultra->vel_hpf ) <= 35)
//											{
//													counter_loop=0;
//												if(z_vel.need_2_res == 1)
//												{
//													z_vel.P =1;
//													z_vel.need_2_res = 0;
//												}
//												ultra_noise=65;
//												Update_Kalman1x1(&z_vel,ultra->vel,acc_z * 100,ultra_noise,3);
//													

//											}
//											
//											else if( 35 < fabs( ultra->vel_hpf ) <250)
//											{
//												counter_loop=25;
//												ultra_noise=1300;// + 3000*(ultra->vel/25);
//												z_vel.need_2_res = 1;
//												Update_Kalman1x1(&z_vel,ultra->vel,acc_z * 100,ultra_noise,3);		
//												
//											}
//											else 
//												{
//													counter_loop=25;
//													z_vel.state = z_vel.state + acc_z * 0.4f;
//												}
//     }
//    if(counter_loop > 0)
//			{
//				if(counter_loop>=1)
//					{
//				   counter_loop--;
//					}
//					
//	     z_vel.state = z_vel.state + acc_z * 0.4f;		
//				}
//************************************************************************

if(fabs( ultra->vel_hpf ) <= 35)
											{
													counter_loop=0;
												if(z_vel.need_2_res == 1)
												{
													z_vel.P =1;
													z_vel.need_2_res = 0;
												}
												ultra_noise=65;
												Update_Kalman1x1(&z_vel,ultra->vel,acc_z * 100,ultra_noise,3);
													

											}
											
											else if( 35 < fabs( ultra->vel_hpf )  && fabs( ultra->vel_hpf ) <50)
											{
												ultra_noise=2900;
												z_vel.need_2_res = 1;
												Update_Kalman1x1(&z_vel,ultra->vel,acc_z * 100,ultra_noise,3);		
												
											}
											else if( 50 <= fabs( ultra->vel_hpf )  && fabs( ultra->vel_hpf ) <60)
											{
												ultra_noise=3600;
												z_vel.need_2_res = 1;
												Update_Kalman1x1(&z_vel,ultra->vel,acc_z * 100,ultra_noise,3);		
												
											}
											else if( 60 <= fabs( ultra->vel_hpf )  && fabs( ultra->vel_hpf ) <250)
											{
												ultra_noise=6000;
												z_vel.need_2_res = 1;
												Update_Kalman1x1(&z_vel,ultra->vel,acc_z * 100,ultra_noise,3);		
												
											}
											else 
												{
													z_vel.state = z_vel.state + acc_z * 0.4f;
												}
												
	//*****************************************************************
												

		
		}
	
	else
	{
		z_vel.state = z_vel.state + acc_z * 0.4f;
	}	
	
	return 1.0;
}

