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

		if (rc.height_ctrl_mode == 1)
		{
			height_ctrl_outer(T, thr, z_height, wz_speed);
			height_speed_ctrl(T, thr, ultra_ctrl_out, wz_speed);

		}

		if (rc. height_ctrl_mode == 2)
		{
			                    
				//height_speed_ctrl(0.021f, thr, 0.4f*ultra_ctrl_out, ultra_speed);
			 
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
	 

	height_thr = LIMIT(thr, 0, THR_BEFOR_FLY_UP);

	thr_lpf += (1 / (1 + 1 / (0.5f *3.14f *T))) *(height_thr - thr_lpf);
 
	float w_kp_weight = 1;

	 
	wz_speed_pid_v.err = (exp_z_speed - wz_speed);

	wz_speed_pid_v.err_d = wz_speed_pid.kd *0.3f*(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
	wz_speed_pid_v.err_d = LIMIT(wz_speed_pid_v.err_d, -ctrl_s.Thr_Weight * 40, ctrl_s.Thr_Weight * 40);

	wz_speed_pid_v.err_i += w_kp_weight* wz_speed_pid.ki *0.3f*(wz_speed_pid_v.err + exp_z_speed)  *T;
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i, (-ctrl_s.Thr_Weight * 100), (ctrl_s.Thr_Weight * 100));

	wz_speed_pid_v.pid_out = thr_lpf + ctrl_s.Thr_Weight *LIMIT((wz_speed_pid.kp *(exp_z_speed + wz_speed_pid_v.err) + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i), -200, 200);

	wz_speed_pid_v.err_old = wz_speed_pid_v.err;
}

 
 

void HLT_CTRL::height_ctrl_outer(float T, float thr, float height,float speed)
{
	float aim_height;

	exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - THR_BEFOR_FLY_UP, 50) / 300.0f; //+-ULTRA_SPEEDmm / s
	exp_height_speed = LIMIT(exp_height_speed, -ULTRA_SPEED, ULTRA_SPEED);
	aim_height = ULTRA_MAX_HEIGHT*(thr - THR_BEFOR_FLY_UP) / (1000 - THR_BEFOR_FLY_UP);


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
	exp_height = LIMIT(exp_height, 50, aim_height);

	ultra_ctrl.err = ((exp_height - height));
	float kp_weight = 1;
	 

	float ki_weight = 1.0f;
	 

	ultra_ctrl.err_i += ki_weight* ultra_ctrl.err *T;

	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i, (-ctrl_s.Thr_Weight *ULTRA_INT), (ctrl_s.Thr_Weight *ULTRA_INT));

	//ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(ultra_ctrl.err - ultra_ctrl.err_old) );
	//ultra_ctrl.err_d  = ( ultra_pid.kd *0.12f * (ultra_ctrl.err - ultra_ctrl.err_old) );

	ultra_ctrl.err_d = (-ultra_pid.kd *0.12f *  speed);
	ultra_ctrl.err_d = LIMIT(ultra_ctrl.err_d, -10.0f, 10.0f);

	ultra_ctrl.pid_out = kp_weight*ultra_pid.kp*ultra_ctrl.err + ultra_pid.ki * ultra_ctrl.err_i + ultra_ctrl.err_d;

	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out, -500, 500);

	ultra_ctrl_out = ultra_ctrl.pid_out;

	ultra_ctrl.err_old = ultra_ctrl.err;


}

/*---------与高度传感器融合得到高度、速度--------*/
void HLT_CTRL::speed_height_caculate_with_sensor(float T, float sensor_height, float sensor_speed)
{
	static float hc_acc_i, wz_speed_0, wz_speed_1;
//	static float z_height_loop_hz= 0.2;
//	static float z_height_loop_hz1 =0.1;
//	static float z_height_loop_hz2 = 0.1;

	wz_acc += (1 / (1 + 1 / (2.0f *3.14f *T))) *((imu_dcm.reference_v.z *ahrs.Acc.z + imu_dcm.reference_v.x * ahrs.Acc.x + imu_dcm.reference_v.y * ahrs.Acc.y - 4096 * 2) - wz_acc);
	wz_acc_mms2 = (wz_acc / 4096.0f / 2.0) * 10000 + hc_acc_i;//9800 *T;

	wz_speed_0 += my_deathzoom((wz_acc_mms2), 100) *T;

	hc_acc_i += 0.1f *T *((wz_speed - wz_speed_old) / T - wz_acc_mms2);
	hc_acc_i = LIMIT(hc_acc_i, -500, 500);


	wz_speed_0 += (1 / (1 + 1 / (0.1f *3.14f *T))) *(sensor_speed - wz_speed_0);

	wz_speed_1 = wz_speed_0;

	if (ABS(wz_speed_1) < 0)
	{
		wz_speed_1 = 0;
	}

	wz_speed = wz_speed_1;

	wz_speed_old = wz_speed;
	

	z_height += wz_speed*T;
	z_height += (1 / (1 + 1 / (0.2f *3.14f *T))) *(sensor_height - z_height);
	//z_height =   (z_height_old + wz_speed*T);
	z_height_old = z_height;

}

