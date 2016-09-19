#include "height_ctrl.h"
#include "scheduler.h"
#include "mymath.h"
#include "filter.h"
#include "ctrl.h"
#include "AHRS.h"
#include "rc.h"
#include "imu.h"
#include "sonar.h"

HLT_CTRL hlt_ctl;

void HLT_CTRL:: WZ_Speed_PID_Init()
{
	wz_speed_pid.kp = 0.55f;
	wz_speed_pid.kd = 3.35;
	wz_speed_pid.ki = 0.27;
}

void HLT_CTRL:: Ultra_PID_Init()
{
	ultra_pid.kp = 0.09;
	ultra_pid.kd = 0.8;
	ultra_pid.ki = 0.05;
}

void HLT_CTRL:: Height_Ctrl(float T, float thr)
{
	static float wz_speed_t;
	static u8 height_ctrl_start_f;
	static u16 hc_start_delay;
	static u8 hs_ctrl_cnt;

	switch (height_ctrl_start_f)
	{
	case 0:

		if (ahrs.Acc.z > 8000)
		{
			height_ctrl_start_f = 1;
		}
		else if (++hc_start_delay > 500)
		{
			height_ctrl_start_f = 1;
		}

		break;

	case 1:


		wz_acc += (1 / (1 + 1 / (2.0f *3.14f *T))) *((imu_dcm. reference_v.z *ahrs.Acc.z + imu_dcm. reference_v.x * ahrs.Acc.x + imu_dcm. reference_v.y * ahrs.Acc.y - 4096 * 2) - wz_acc);



		wz_speed_t += (1 / (1 + 1 / (0.5f *3.14f *T))) *(0.4f*(thr - 420) - wz_speed_t);

		// Moving_Average( (float)( baro_alt_speed *10),baro_speed_arr,BARO_SPEED_NUM, baro_cnt ,&baro_speed ); //单位mm/s
		// 		if( baro_alt_speed > 2000 )
		// 		{
		// 			while(1);
		// 		}
		//		
		//		if( height_ctrl_mode == 1)
		//		{
		//			//height_speed_ctrl(T,thr,0.8f*(thr-500),wz_speed_t);
		//			
		//			
		//			if(baro_ctrl_start==1)//20ms
		//			{
		//				height_speed_ctrl(0.02f,thr,( EXP_Z_SPEED ),baro_speed);//baro_alt_speed *10);///
		//				baro_ctrl_start = 0;
		//				Baro_Ctrl(0.02f,thr);
		//			}		
		//		}




		if (rc. height_ctrl_mode == 2)
		{
			hs_ctrl_cnt++;
			hs_ctrl_cnt = hs_ctrl_cnt % 7;
			if (hs_ctrl_cnt == 0)
			{

				//                          static  float height_loop_time;
				//                          
				//                          height_loop_time = Get_Cycle_T(3);
				height_speed_ctrl(0.021f, thr, 0.4f*ultra_ctrl_out, ultra_speed);
			}

			//			if( ultra_start_f == 0 )
			//			{	
			//				
			//				Ultra_Ctrl(0.1f,thr);//超声波周期100ms
			//				ultra_start_f = -1;
			//			}
		}


		////////////////////////////////////////////////////////////	
		if (rc.height_ctrl_mode)
		{
			height_ctrl_out = wz_speed_pid_v.pid_out;
		}
		else
		{
			height_ctrl_out = thr;
		}

		break; //case 1

	default: break;

	} //switch
}

void HLT_CTRL:: height_speed_ctrl(float T, float thr, float exp_z_speed, float h_speed)
{
	static float thr_lpf;
	float height_thr;
	static float hc_acc_i, wz_speed_0, wz_speed_1;

	height_thr = LIMIT(thr, 0, 545);

	thr_lpf += (1 / (1 + 1 / (0.5f *3.14f *T))) *(height_thr - thr_lpf);

	wz_acc_mms2 = (wz_acc / 4096.0f / 2.0) * 10000 + hc_acc_i;//9800 *T;





	wz_speed_0 += my_deathzoom((wz_acc_mms2), 100) *T;

	hc_acc_i += 0.4f *T *((wz_speed - wz_speed_old) / T - wz_acc_mms2);
	hc_acc_i = LIMIT(hc_acc_i, -500, 500);


	wz_speed_0 += (1 / (1 + 1 / (0.5f *3.14f *T))) *(h_speed - wz_speed_0);

	wz_speed_1 = wz_speed_0;

	if (ABS(wz_speed_1) < 0)
	{
		wz_speed_1 = 0;
	}

	wz_speed = wz_speed_1;

	wz_speed_old = wz_speed;

	float w_kp_weight = 1;

	if (ultra_ctrl.err > 100)
	{
		w_kp_weight = 1.0 + 1.0*(ultra_ctrl.err - 200) / 500;
		w_kp_weight = LIMIT(w_kp_weight, 1.0, 3.0);

	}
	else
	{
		w_kp_weight = 1.0;
	}

	wz_speed_pid_v.err = (exp_z_speed - wz_speed);

	wz_speed_pid_v.err_d = wz_speed_pid.kd *0.3f*(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
	wz_speed_pid_v.err_d = LIMIT(wz_speed_pid_v.err_d, -ctrl_s.Thr_Weight * 40, ctrl_s.Thr_Weight * 40);

	wz_speed_pid_v.err_i += w_kp_weight* wz_speed_pid.ki *0.3f*(wz_speed_pid_v.err + exp_z_speed)  *T;
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i, (-ctrl_s.Thr_Weight * 100), (ctrl_s.Thr_Weight * 100));

	wz_speed_pid_v.pid_out = thr_lpf + ctrl_s.Thr_Weight *LIMIT((wz_speed_pid.kp *(exp_z_speed + wz_speed_pid_v.err) + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i), -200, 200);

	wz_speed_pid_v.err_old = wz_speed_pid_v.err;
}


void HLT_CTRL:: Ultra_speed_caculate(float T)
{

	ultra_sp_tmp = Moving_Median(0, 5, sonar.ultra_delta / T); //ultra_delta/T;
	ultra_dis_tmp = Moving_Median(1, 5, sonar.ultra_distance);
	//	
	//ultra_sp_tmp =  ultra_delta/T/5 ; //ultra_delta/T;
	//ultra_dis_tmp = ultra_distance;

	if (ultra_dis_tmp < 2000)
	{
		if (ABS(ultra_sp_tmp) < 100)
		{
			ultra_speed += (1 / (1 + 1 / (4 * 3.14f *T))) * ((float)(ultra_sp_tmp)-ultra_speed);
		}
		else
		{
			ultra_speed += (1 / (1 + 1 / (1.0f *3.14f *T))) * ((float)(ultra_sp_tmp)-ultra_speed);
		}
	}

	ultra_speed = LIMIT(ultra_speed, -300, 300);
	//	if( ultra_dis_tmp < 2000 )
	//	{
	//		
	//		if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
	//		{
	//			
	//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
	//		}
	//		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
	//		{
	//			
	//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
	//		}
	//		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
	//		{
	//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
	//		}
	//		else
	//		{
	//			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
	//		}
	//	}
	//	else
	//	{
	//		
	//	}

	//换超声波时记得用这句话
	ultra_dis_lpf = ultra_dis_tmp;

}


void HLT_CTRL:: Ultra_Ctrl(float T, float thr)
{


	exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 450, 50) / 300.0f; //+-ULTRA_SPEEDmm / s
	exp_height_speed = LIMIT(exp_height_speed, -ULTRA_SPEED, ULTRA_SPEED);

	if (exp_height > ULTRA_MAX_HEIGHT)
	{
		if (exp_height_speed > 0)
		{
			exp_height_speed = 0;
		}
	}
	else if (exp_height < 20)
	{
		if (exp_height_speed < 0)
		{
			exp_height_speed = 0;
		}
	}

	exp_height += exp_height_speed *T;
	exp_height = LIMIT(exp_height, 300, ULTRA_MAX_HEIGHT);


	if (thr < 100)
	{
		exp_height += (1 / (1 + 1 / (0.2f *3.14f *T))) *(-exp_height);
	}


	ultra_ctrl.err = ((exp_height - ultra_dis_lpf));
	float kp_weight = 1;


	 
		if (ultra_ctrl.err > 200)
		{

			kp_weight = 1.0f + 1.0f*(ultra_ctrl.err - 200) / 50;
			kp_weight = LIMIT(kp_weight, 1.0f, 3.0f);

		}
		else
		{
			kp_weight = 1.0;
		}
	

	float ki_weight = 1.0f;
	if (ultra_ctrl.err > 100)
	{

		ki_weight = 1.0f + 1.0f*(ultra_ctrl.err - 200) / 50;
		ki_weight = LIMIT(kp_weight, 1.0f, 3.0f);

	}
	else
	{
		ki_weight = 1.0;
	}

	ultra_ctrl.err_i += ki_weight* ultra_ctrl.err *T;

	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i, (-ctrl_s.Thr_Weight *ULTRA_INT), (ctrl_s.Thr_Weight *ULTRA_INT));

	//ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	//ultra_ctrl.err_d  = ( ultra_pid.kd *0.12f * (ultra_ctrl.err - ultra_ctrl.err_old) );

	ultra_ctrl.err_d = (-ultra_pid.kd *0.12f *  ultra_speed);
	ultra_ctrl.err_d = LIMIT(ultra_ctrl.err_d, -10.0f, 10.0f);

	ultra_ctrl.pid_out = kp_weight*ultra_pid.kp*ultra_ctrl.err + ultra_pid.ki * ultra_ctrl.err_i + ultra_ctrl.err_d;

	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out, -500, 500);

	ultra_ctrl_out = ultra_ctrl.pid_out;

	ultra_ctrl.err_old = ultra_ctrl.err;
}
