#include "pos_ctrl.h"
#include "mymath.h"
#include "rc.h"
#include "gps_kalman.h"
#include "position.h"
#include "mymath.h"
#include "gps_kalman_v4.h"
#include "imu_ekf2.h"
#include "ctrl.h"
#include "filter.h"
#define  MAX_POS_ERR 3000    //mm/s
#define  MAX_SPEED_ERR 1000    //mm/s
#define  MAX_SPEED_CTRL_ANGLE 10.5f    //角度输出限幅  
#define  MAX_POS_CTRL_SPEED 3000    //位置输出速度 
#define  MAX_SPEED_CTRL_ACC 5000    //位置目标加速度
#define  NEED_ANTI_FULL_OF_INTERGRAL 0
#define  NEAR_CENTER_DISTANCE  0     //mm
#define  LOSE_CENTER_DITANCE  0   //位置控制死区圆环式
Pos_CTRL pos_ctrl;


/************************************************************************/
/* 位置pid输出目标速度                                                                     */
/************************************************************************/
float POS_Ctrl_PID::pos_hold_pid(float T, float thr_weight, float aim_pos, float actual_pos, float actual_v)
{
	err[2] = err[1];
	err[1] = err[0];
	err[0] =  aim_pos - actual_pos;
	float ABS_ERR = ABS(err[0]);

	if (ABS_ERR<NEAR_CENTER_DISTANCE) //到达中心之后把死区放大
	{
		position_death_room = LOSE_CENTER_DITANCE;
	}
	else if (ABS_ERR>LOSE_CENTER_DITANCE) //跑出去之后把死区关掉
	{
		position_death_room = 0;
	}
	err[0] = my_deathzoom(err[0], position_death_room);
	err[0] = LIMIT(err[0], (-MAX_POS_ERR), MAX_POS_ERR);
	//分段
	float  kp_weight = 1;
	float  ki_weight = 1.0f;
	float ABS_ULT_err = ABS_ERR;
	/*float kp_weight_diff_point_mm = kp_weight_diff_point * 1000;
	if (ABS_ULT_err > kp_weight_diff_point_mm)
	{
		 kp_weight = 1.0f + kp_weight_amplify*
			(ABS_ULT_err - kp_weight_diff_point_mm)*(ABS_ULT_err - kp_weight_diff_point_mm) / 10000;
		 kp_weight = LIMIT( kp_weight, 1.0f, max_kp_weight);
		 ki_weight = kp_weight_diff_point_mm / ABS_ULT_err;
		 ki_weight = LIMIT( ki_weight, 0.1f, 1.0f);
	}*/
	//模糊调参
	/*pos_fuzzy_P.UFF[4] = chang_pos_fuzzy*100;
	pos_fuzzy_P.UFF[5] = chang_pos_fuzzy*100+20;
	pos_fuzzy_P.UFF[6] = chang_pos_fuzzy*100+40;*/

    //模糊分段

	kp_weight = ((float)pos_fuzzy_P.fuzzy_paramameter_output(err[0], -actual_v)) / 100.0f;
	kp_weight = LIMIT(kp_weight, 0.5f, max_kp_weight);
	kp_weight_amplify = kp_weight;
	//分段结束


	P =kp_weight* Kp*err[0];
	P = LIMIT(P, (-MAX_POS_CTRL_SPEED), MAX_POS_CTRL_SPEED);

	Ii +=ki_weight* Ki*err[0] * T;
	Ii = LIMIT(Ii, (-0.3*thr_weight*MAX_POS_CTRL_SPEED), 0.3*thr_weight*MAX_POS_CTRL_SPEED);

	D = Kd*(
		0.6f*(err[0] - err[1]) + 0.4f*(err[1] - err[2])
		);
	D = LIMIT(D, (-0.5*MAX_POS_CTRL_SPEED), 0.5*MAX_POS_CTRL_SPEED);
	//组合输出 
	pid_out = P + Ii + D;
	pid_out = LIMIT(pid_out, -MAX_POS_CTRL_SPEED, MAX_POS_CTRL_SPEED);
	return pid_out;


}
/************************************************************************/
/* 速度PID，输出角度                                                                     */
/************************************************************************/
float Speed_Ctrl_PID::  speed_hold_pid(float T, float thr_weight, float aim_speed, float actual_speed)
{
        err[5] = err[4];
	err[4] = err[3];
	err[3] = err[2];
	err[2] = err[1];
	err[1] = err[0];
	err[0] = aim_speed - actual_speed;
	//调用LQR反馈控制器
//	lqr_ctler.z_function_discreate_ctrl(100);
	//lqr_ctler.out = LIMIT(lqr_ctler.out, -lqr_ctler.thi_limit, lqr_ctler.thi_limit);
	//误差限幅
	//err[0] = LIMIT(err[0], (-MAX_SPEED_ERR), MAX_SPEED_ERR);

    P =10.0f* Kp*err[0];
	P = LIMIT(P, (-MAX_SPEED_CTRL_ACC), MAX_SPEED_CTRL_ACC);
	/************************************************************************/
	/*     积分                                                                 */
	/************************************************************************/
//#if(NEED_ANTI_FULL_OF_INTERGRAL)
//	if (pid_out >= MAX_SPEED_CTRL_ANGLE)
//	{
//		if (err[0] < 0)
//		{
//			I += Ki*0.1f*err[0] * T;
//		}
//
//	}
//	else if (pid_out <= (-MAX_SPEED_CTRL_ANGLE))
//	{
//		if (err[0] > 0)
//		{
//			I += Ki*0.1f*err[0] * T;
//		}
//	}
//	else
//	{
//		I += Ki*0.1f*err[0] * T;
//	}
//
//#else //不需要积分清零
	 Ii += 10.0f*Ki *err[0] * T;
//#endif
 
	 Ii = LIMIT(Ii, (-thr_weight*MAX_SPEED_CTRL_ACC), thr_weight*MAX_SPEED_CTRL_ACC);
	/************************************************************************/
	/*    积分结束                                                                  */
	/************************************************************************/
	D = (Kd)* (
		0.6f*(err[0] - err[1]) + 0.4f*(err[1] - err[2])
		+ 0.4f*(err[2] - err[3]) + 0.3f*(err[3] - err[4])
		);
	D = LIMIT(D, (-0.5f*MAX_SPEED_CTRL_ACC), 0.5f*MAX_SPEED_CTRL_ACC);
	//组合输出 
	pid_out = P + Ii + D;
	pid_out = LIMIT(pid_out, -10000, 10000);
	return pid_out;
}


/************************************************************************/
/* 加速度PID，输出角度                                                                     */
/************************************************************************/
float Acc_HLOD_PID::acc_hold_pid(float T, float thr_weight, float aim_acc, float actual_acc)
{
	err[5] = err[4];
	err[4] = err[3];
	err[3] = err[2];
	err[2] = err[1];
	err[1] = err[0];
	//err[0] = aim_acc - actual_acc;
	err[0] = 0 - actual_acc; //全程 0 acc控制
	//误差限幅
	//err[0] = LIMIT(err[0], (-MAX_SPEED_ERR), MAX_SPEED_ERR);

	P =0.1* Kp*(0.5*err[0]+0.5*err[1]);
	P = LIMIT(P, (-0.5f), 0.5f);
	/************************************************************************/
	/*     积分                                                             */
	/************************************************************************/
	Ii += Ki*0.01f*err[0] * T;
	Ii = LIMIT(Ii, (-thr_weight*1.0f), thr_weight*1.0f);
	/************************************************************************/
	/*    积分结束                                                                  */
	/************************************************************************/
	D = Kd*0.01f*(
		0.6f*(err[0] - err[1]) + 0.4f*(err[1] - err[2])
		//+ 0.4f*(err[2] - err[3]) + 0.3f*(err[3] - err[4]) + 0.3f*(err[4] - err[5])
		);
	D = LIMIT(D, (-0.5f), 0.5f);
	//组合输出 
	pid_out =0.001* aim_acc + P + Ii + D;
	pid_out = LIMIT(pid_out, -MAX_SPEED_CTRL_ANGLE, MAX_SPEED_CTRL_ANGLE);
	return pid_out;
}


/************************************************************************/
/* 参数清零                                                                     */
/************************************************************************/
void Pos_CTRL::para_clear()
{
	x_v_ctrl.lqr_ctler.out = 0;
	y_v_ctrl.lqr_ctler.out = 0;
	out_pitch = 0;
	out_roll = 0;
	out_speed.x = 0;
	out_speed.y = 0;
	x_pos_ctrl.Ii = 0;
	x_v_ctrl.Ii = 0;
	y_pos_ctrl.Ii = 0;
	y_v_ctrl.Ii = 0;
	x_acc_ctrl.Ii = 0;
	y_acc_ctrl.Ii = 0;
	/*   aim_position.x = position.x_pos_hf;
	   aim_position.y = position.y_pos_hf;*/
	//aim_position.x = gps.gps_pos_x_b * 1000;
	//aim_position.y = gps.gps_pos_y_b * 1000;
	/*aim_position.x = gps_po_kf_v4.x_b;
	aim_position.y = gps_po_kf_v4.y_b;*/
	aim_position.x = gps.px*1000;
	aim_position.y = gps.py*1000;

}

/************************************************************************/
/* 位置控制总行数                                                                     */
/************************************************************************/
void Pos_CTRL::position_ctrl(float T, float thr_weight, 
	float x_v_actual, float y_v_actual,
	float x_pos_actual, float y_pos_actual) 

{
	//由目标位置输出速度(实际YAW角决定)
	/*aim_position_b.x = aim_position.x*cos(imu_ekf2.yaw*M_PI_F / 180.0f) + aim_position.y *sin(imu_ekf2.yaw*M_PI_F / 180.0f);
	aim_position_b.y = -aim_position.x*sin(imu_ekf2.yaw*M_PI_F / 180.0f) + aim_position.y *cos(imu_ekf2.yaw*M_PI_F / 180.0f);
*/

	//目标位置改变
	if (ABS(rc.CH_filter[PIT])>150)
	{
		aim_position.x = gps.px * 1000;
	}
	if (ABS(rc.CH_filter[ROL]) > 150)
	{
		aim_position.y = gps.py * 1000;
	}
	//目标YAW角决定
	aim_position_b.x = aim_position.x*cos(ctrl_s.except_A.z*M_PI_F / 180.0f) + aim_position.y *sin(ctrl_s.except_A.z*M_PI_F / 180.0f);
	aim_position_b.y = -aim_position.x*sin(ctrl_s.except_A.z*M_PI_F / 180.0f) + aim_position.y *cos(ctrl_s.except_A.z*M_PI_F / 180.0f);
	out_speed.x = x_pos_ctrl.pos_hold_pid(T, thr_weight, aim_position_b.x, x_pos_actual, x_v_actual);
	out_speed.y = y_pos_ctrl.pos_hold_pid(T, thr_weight, aim_position_b.y, y_pos_actual, y_v_actual);
	//遥控目标速度
	thr_aim_v.x =10* my_deathzoom((rc. CH_filter[PIT]), 150);
	thr_aim_v.y = -10*my_deathzoom((rc.CH_filter[ROL]), 150);
	//复合目标速度
	if (Pos_to_speed_on) //需要外环
	{
		aim_v.x = out_speed.x + thr_aim_v.x;
		aim_v.y =out_speed.y + thr_aim_v.y;
	} 
	else
	{
		aim_v.x = 0.0f + thr_aim_v.x;
		aim_v.y = 0.0f + thr_aim_v.y;
	}

}

void Pos_CTRL::speed_ctrl(float T, float thr_weight, float x_acc_actual, float y_acc_actual, float x_v_actual, float y_v_actual)
{
	//速度PID
	//out_pitch = x_v_ctrl.speed_hold_pid(T, thr_weight, aim_v.x, x_v_actual);
	//out_pitch = -out_pitch;//配合飞机进行反向
	//out_roll = y_v_ctrl.speed_hold_pid(T, thr_weight, aim_v.y, y_v_actual);

	x_v_ctrl.speed_hold_pid(T, thr_weight, aim_v.x, x_v_actual);
	y_v_ctrl.speed_hold_pid(T, thr_weight, aim_v.y, y_v_actual);
	aim_acc.x = x_v_ctrl.pid_out;
	aim_acc.y = y_v_ctrl.pid_out;
	acc_ctrl(T, thr_weight, x_acc_actual, y_acc_actual);

}
void Pos_CTRL::acc_ctrl(float T, float thr_weight, float x_acc_actual, float y_acc_actual)
{
	x_acc_ctrl.acc_hold_pid(T, thr_weight, aim_acc.x, x_acc_actual);
	y_acc_ctrl.acc_hold_pid(T, thr_weight, aim_acc.y, y_acc_actual);
	//out_pitch = -x_acc_ctrl.pid_out;
	//out_roll  =  y_acc_ctrl.pid_out;
	//二阶低通滤波
	//叠加LQR控制器输出
	x_acc_ctrl.pid_out = -x_acc_ctrl.pid_out;//- x_v_ctrl.lqr_ctler.out;
	//y_acc_ctrl.pid_out = y_acc_ctrl.pid_out + y_v_ctrl.lqr_ctler.out;
	filter.IIR_second_lf(T, &x_acc_ctrl.pid_out, &out_pit_fil1, &out_pitch, out_filter1, out_filter2);
	filter.IIR_second_lf(T, &y_acc_ctrl.pid_out, &out_roll_fil1, &out_roll, out_filter1, out_filter2);
//	out_pitch = -out_pitch;
}
