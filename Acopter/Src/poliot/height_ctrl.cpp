#include "height_ctrl.h"
#include "scheduler.h"
#include "mymath.h"
#include "filter.h"
#include "ctrl.h"
#include "AHRS.h"
#include "rc.h"
#include "imu.h"
#include "sonar.h"
#include "baro.h"
#include "gps.h"
#include "imu_ekf2.h"
#define MAX_HEIGH_PID_OUT 500
#define  MAX_ERR_BETEEN_ACTH_EX_H  400  //目标与实际的之间的差距mm
#define  NEED_HEIGHT_ITER_CLERA 1  // 外环抗饱和
#define  NEED_OUTER_FUZZY    1
#define  OPEN_ACC_CTRL  1

#define  NEED_SPEED_ITER_CLERA 1  //内环抗饱和
#define  MAX_SPEED_PID_OUT  2000
#define  MAX_ACC_PID_OUT  200
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

	 

		////////////////////////////////////////////////////////////	
		if (rc.height_ctrl_mode)
		{
			height_ctrl_out = wz_speed_pid_v.pid_out;
			//打开加速度控制
			#if  OPEN_ACC_CTRL
			height_ctrl_out = acc_pid_v.pid_out;
			#endif

		}
		else
		{
			height_ctrl_out = thr;
		}

		break; //case 1

	default: break;

	} //switch
}
void HLT_CTRL::height_acc_ctrl(float T, float thr, float exp_z_acc, float z_acc)
{

	acc_pid_v.err = (exp_z_acc - z_acc);
	//acc_pid_v.err = LIMIT(acc_pid_v.err, -500, 500);
	//不完全微分
	for (short i = 10; i >= 1; i--)
	{
		acc_pid_v.err_d_save[i] = acc_pid_v.err_d_save[i - 1];
	}
	acc_pid_v.err_d_save[0] = acc_pid.kd *0.3f*(acc_pid_v.err - acc_pid_v.err_old);
	acc_pid_v.err_d = 0.3f*acc_pid_v.err_d_save[0] + 0.3f*acc_pid_v.err_d_save[1]
		+ 0.2f*acc_pid_v.err_d_save[2] + 0.2f*acc_pid_v.err_d_save[3];
	/*+ 0.2f*acc_pid_v.err_d_save[4] + 0.2f*acc_pid_v.err_d_save[5]
	+ 0.2f*acc_pid_v.err_d_save[6] + 0.2f*acc_pid_v.err_d_save[7];
	+0.2f*acc_pid_v.err_d_save[8] + 0.2f*acc_pid_v.err_d_save[9];
	+0.2f*acc_pid_v.err_d_save[10] + 0.2f*acc_pid_v.err_d_save[11];*/
	//不完全微分结束

	acc_pid_v.err_d = LIMIT(acc_pid_v.err_d, -ctrl_s.Thr_Weight * 40, ctrl_s.Thr_Weight * 40);
	acc_pid_v.err_i +=   acc_pid.ki *0.3f* acc_pid_v.err  *T;
	acc_pid_v.err_i = LIMIT(acc_pid_v.err_i, (-ctrl_s.Thr_Weight * 200), (ctrl_s.Thr_Weight * 200));
	/****************积分解算结束********************************************************/

	acc_pid_v.pid_out_err = ctrl_s.Thr_Weight
		*LIMIT((acc_pid.kp *acc_pid_v.err + acc_pid_v.err_d + acc_pid_v.err_i),
		-MAX_ACC_PID_OUT, MAX_ACC_PID_OUT);
	acc_pid_v.pid_out = thr_lpf + acc_pid_v.pid_out_err;
	acc_pid_v.err_old = acc_pid_v.err;


}


void HLT_CTRL:: height_speed_ctrl(float T, float thr, float exp_z_speed, float h_speed)
{

	float height_thr;
	 

	height_thr = LIMIT(thr, 0, THR_BEFOR_FLY_UP);
	thr_lpf += (1 / (1 + 1 / (0.5f *3.14f *T))) *(height_thr - thr_lpf);
	/************************************************************************/
	/* 进行分段                                                              */
	/************************************************************************/

	float wz_kp_weight = 1;
	float wz_ki_weight = 1.0f;
	 
	wz_speed_pid_v.err = (exp_z_speed - h_speed);
	//wz_speed_pid_v.err = LIMIT(wz_speed_pid_v.err, -500, 500);
	//不完全微分
	for (short i = 10; i >= 1; i--)
	{
		wz_speed_pid_v.err_d_save[i] = wz_speed_pid_v.err_d_save[i - 1];
	}
	wz_speed_pid_v.err_d_save[0] = wz_speed_pid.kd *0.3f*(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
	wz_speed_pid_v.err_d = 0.3f*wz_speed_pid_v.err_d_save[0] + 0.3f*wz_speed_pid_v.err_d_save[1]
		+ 0.2f*wz_speed_pid_v.err_d_save[2] + 0.2f*wz_speed_pid_v.err_d_save[3];
		/*+ 0.2f*wz_speed_pid_v.err_d_save[4] + 0.2f*wz_speed_pid_v.err_d_save[5]
		+ 0.2f*wz_speed_pid_v.err_d_save[6] + 0.2f*wz_speed_pid_v.err_d_save[7];
	    +0.2f*wz_speed_pid_v.err_d_save[8] + 0.2f*wz_speed_pid_v.err_d_save[9];
	    +0.2f*wz_speed_pid_v.err_d_save[10] + 0.2f*wz_speed_pid_v.err_d_save[11];*/
	//不完全微分结束
	 
	wz_speed_pid_v.err_d = LIMIT(wz_speed_pid_v.err_d, -ctrl_s.Thr_Weight * 40, ctrl_s.Thr_Weight * 40);


	/************************************************************************/
	/* 是否需要积分清零                                                                     */
	/************************************************************************/
	float err_Intergral = wz_speed_pid_v.err + exp_z_speed;
#if(NEED_SPEED_ITER_CLERA)
	if (wz_speed_pid_v.pid_out_err >= MAX_SPEED_PID_OUT)
	{
		if (err_Intergral < 0)
		{
			wz_speed_pid_v.err_i += wz_ki_weight* wz_speed_pid.ki *0.3f*err_Intergral *T;
		}
	}
	else if (wz_speed_pid_v.pid_out_err <= (-MAX_SPEED_PID_OUT))
	{
		if (err_Intergral > 0)
		{
			wz_speed_pid_v.err_i += wz_ki_weight* wz_speed_pid.ki *0.3f*err_Intergral  *T;
		}
	}
	else
	{
		wz_speed_pid_v.err_i += wz_ki_weight* wz_speed_pid.ki *0.3f*err_Intergral  *T;
	}

#else //不需要积分清零
	wz_speed_pid_v.err_i += w_kp_weight* wz_speed_pid.ki *0.3f*err_Intergral  *T;
#endif
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i, (-ctrl_s.Thr_Weight * 200), (ctrl_s.Thr_Weight * 200));
/****************积分解算结束********************************************************/

	wz_speed_pid_v.pid_out_err = ctrl_s.Thr_Weight 
		*LIMIT((wz_kp_weight* wz_speed_pid.kp *wz_speed_pid_v.err  + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),
		-MAX_SPEED_PID_OUT, MAX_SPEED_PID_OUT);
	wz_speed_pid_v.pid_out = thr_lpf + wz_speed_pid_v.pid_out_err;
	wz_speed_pid_v.err_old = wz_speed_pid_v.err;
//打开加速度控制
#if  OPEN_ACC_CTRL
	height_acc_ctrl(T, thr, wz_speed_pid_v.pid_out_err, wz_acc_mms2);
#endif
}

 
 

void HLT_CTRL::height_ctrl_outer(float T, float thr, float height,float speed)
{
	float aim_height_initial;
	 
	exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - THR_BEFOR_FLY_UP, 50) / 300.0f; //+-ULTRA_SPEEDmm / s
	//exp_height_speed = LIMIT(exp_height_speed, -ULTRA_SPEED, ULTRA_SPEED);
	exp_height_speed = LIMIT(exp_height_speed, -300, 300);


	aim_height_initial = ULTRA_MAX_HEIGHT*(thr - THR_BEFOR_FLY_UP) / (1000 - THR_BEFOR_FLY_UP);
	aim_height_initial = LIMIT(aim_height_initial, 0, ULTRA_MAX_HEIGHT);
	aim_height += T*speed_lp_test *(aim_height_initial - aim_height);

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
	ultra_ctrl.err = LIMIT(ultra_ctrl.err, -1000, 1000);
	/************************************************************************/
	/* 进行分段                                                                     */
	/************************************************************************/

	float kp_weight = 1;
	float ki_weight = 1.0f;
	//float ABS_ULT_err = ABS(ultra_ctrl.err);
	//float kp_weight_diff_point_mm = kp_weight_diff_point * 1000;
	//if (ABS_ULT_err>kp_weight_diff_point_mm)
	//{
	//	kp_weight = 1.0f + kp_weight_amplify*
	//		(ABS_ULT_err - kp_weight_diff_point_mm)*(ABS_ULT_err - kp_weight_diff_point_mm) / 40000;
	//	kp_weight = LIMIT(kp_weight, 1.0f, max_kp_weight);
	//	ki_weight = kp_weight_diff_point_mm / ABS_ULT_err;
	//	ki_weight = LIMIT(ki_weight, 0.1f, 1.0f);
	//}
	/************************************************************************/
	/* 是否需要积分清零                                                                     */
	/************************************************************************/
	//模糊专家分段
#if NEED_OUTER_FUZZY
	fuzzy_KP.UFF[4] = chang_fuzzy * 100;
	fuzzy_KP.UFF[5] = chang_fuzzy * 100 +20;

	fuzzy_KP.UFF[6] = chang_fuzzy * 100+40;


	fuzzy_kp_weight = (float)(fuzzy_KP.fuzzy_paramameter_output(ultra_ctrl.err, -speed)) / 100;
	kp_weight = LIMIT(fuzzy_kp_weight, 0.8, max_kp_weight);
#endif
	

#if(NEED_HEIGHT_ITER_CLERA)
	if (ultra_ctrl.pid_out >= MAX_HEIGH_PID_OUT)
	{
		if(ultra_ctrl.err<0)
		{
			ultra_ctrl.err_i += ki_weight*ultra_pid.ki * ultra_ctrl.err *T;
	    }
		 
	}
	else if (ultra_ctrl.pid_out <= (-MAX_HEIGH_PID_OUT))
	{
		if (ultra_ctrl.err >0)
		{
			ultra_ctrl.err_i += ki_weight* ultra_pid.ki *ultra_ctrl.err *T;
		}
	}
	else
	{
	 
		ultra_ctrl.err_i += ki_weight*ultra_pid.ki * ultra_ctrl.err *T;
		 
	}

#else //不需要积分清零
	ultra_ctrl.err_i += ki_weight* ultra_ctrl.err *T;
#endif
	


	

	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i, (-ctrl_s.Thr_Weight *MAX_HEIGH_PID_OUT), (ctrl_s.Thr_Weight *MAX_HEIGH_PID_OUT));
 
	//不完全微分
	for (short i = 5; i >=1; i--)
	{
		ultra_ctrl.err_d_save[i] = ultra_ctrl.err_d_save[i - 1];
	}
	ultra_ctrl.err_d_save[0] = (-ultra_pid.kd *0.12f *  speed);
	ultra_ctrl.err_d = 0.4f*ultra_ctrl.err_d_save[0] + 0.3f*ultra_ctrl.err_d_save[1] + 0.3f*ultra_ctrl.err_d_save[2];
	//不完全微分结束

		ultra_ctrl.err_d = LIMIT(ultra_ctrl.err_d, -0.1f*MAX_HEIGH_PID_OUT, 0.1f*MAX_HEIGH_PID_OUT);

	ultra_ctrl.pid_out = kp_weight*ultra_pid.kp*ultra_ctrl.err +  ultra_ctrl.err_i + ultra_ctrl.err_d;

	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out, -MAX_HEIGH_PID_OUT, MAX_HEIGH_PID_OUT);

	ultra_ctrl_out = ultra_ctrl.pid_out;

	ultra_ctrl.err_old = ultra_ctrl.err;


}

/*---------与高度传感器融合得到高度、速度--------*/
void HLT_CTRL::speed_height_caculate_with_sensor(float T )
{
	if (rc.height_ctrl_mode == 1)
	{
		speed_height_caculate_with_baro(T, baro.high_filed, baro.speed_filed);
	}

	if (rc.height_ctrl_mode == 2)
	{
		speed_height_caculate_with_baro(T, baro.high_filed, baro.speed_filed);
		//speed_height_caculate_with_baro_gps(T, baro.high_filed, baro.speed_filed,(-gps.pz*1000.0));
	}
 
}

void HLT_CTRL::speed_height_caculate_with_baro(float T, float baro_height, float baro_speed)
{

	static float hc_acc_i = 0, h_i2 = 0, speed_i = 0, height_i = 0, wz_speed_0, wz_speed_1;

	//加标加速度
	for (int i = 99; i > 0; i--)
	{
		wz_acc_old[i] = wz_acc_old[i - 1];
	 
	}
	wz_acc_old[0] = wz_acc;
	//init_wz_acc = (imu_ekf.C_ned2b_body(2, 2) *ahrs.Acc.z + imu_ekf.C_ned2b_body(2,0) * ahrs.Acc.x + imu_ekf.C_ned2b_body(2, 1)* ahrs.Acc.y - 8200.0f) / 8200.0f* 9810.0f;
	init_wz_acc = -imu_ekf2.line_acc(2, 0) * 1000;
	 
	//init_wz_acc = (imu_ekf.C_ned2b_save(2, 2) *ahrs.Acc.z + imu_ekf.C_ned2b_save(0, 2) * ahrs.Acc.x + imu_ekf.C_ned2b_save(1, 2)* ahrs.Acc.y - 8200.0f) / 8200.0f* 9810.0f;
	//wz_acc += (1 / (1 + 1 / (7.5f *3.14f *T))) *(init_wz_acc - wz_acc);
	wz_acc += (1 / (1 + 1 / (7.5f *3.14f *T))) *(init_wz_acc - wz_acc);

	//wz_acc += (1 / (1 + 1 / (1.5f *3.14f *T))) *((imu_dcm.reference_v.z *ahrs.Acc.z + imu_dcm.reference_v.x * ahrs.Acc.x + imu_dcm.reference_v.y * ahrs.Acc.y - 8200.0f) - wz_acc);

	//加速度积分矫正
	//wz_acc_mms2 = wz_acc  + hc_acc_i;//9800 *T;
	wz_acc_mms2 = wz_acc_old[70]+ hc_acc_i;//9800 *T;
	//将积分矫正之后的加速度低通滤波
	wz_acc_mms2_lp += (1 / (1 + 1 / (2.5f *3.14f *T))) *(wz_acc_mms2 - wz_acc_mms2_lp);
	//速度根据滞后值外推
	forward_z_speed = baro.wz_kf_pos_acc;
	for (int i = 69; i > 0; i--)
	{
		//forward_z_speed += my_deathzoom((wz_acc_old[i]), 60) *T;   
		forward_z_speed += wz_acc_old[i]  *T;
	}
	//积分获得速度
	wz_speed_0 += my_deathzoom((wz_acc_mms2), acc_death) *T;
	//向传感器sensor逼近
	wz_speed_0 += (1 / (1 + 1 / (speed_lp_hz2 *3.14f *T)))
		*(my_deathzoom(baro_speed, wz_death) - wz_speed_0);
	//速度误差积分
	speed_i += z_height_ki_2*T*(baro_speed - wz_speed_1);
	speed_i = LIMIT(speed_i, -500, 500);
	//速度误差积分矫正
	wz_speed_1 = wz_speed_0 + h_i2 + speed_i;
	if (ABS(wz_speed_1) < 0)
	{
		wz_speed_1 = 0;
	}
	wz_speed = wz_speed_1;
	//加速度速度误差积分
	hc_acc_i += z_height_ki *T *((wz_speed - wz_speed_old) / T - wz_acc_mms2);
	hc_acc_i = LIMIT(hc_acc_i, -500, 500);
	wz_speed_old = wz_speed;
	z_height += wz_speed*T;
	z_height += (1 / (1 + 1 / (z_height_loop_hz1 *3.14f *T))) *(baro_height - z_height);
	//高度误差积分
	height_i += height_ki*T*(baro_height - z_height);
	height_i = LIMIT(height_i, -200, 200);
	z_height += height_i;
	//速度误差积分
	h_i2 += z_height_ki_2 *T *((z_height - z_height_old) / T - wz_speed_1);
	h_i2 = LIMIT(h_i2, -500, 500);
	z_height_old = z_height;


}
 