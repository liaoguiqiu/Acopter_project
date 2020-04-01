#ifndef _POS_CTRL_H
#define _POS_CTRL_H

#include "include.h" 
#include "vector3.h"
#include "fuzzy.h"
#include "LQR_ctrl.hpp"
struct   POS_Ctrl_PID
{
	POS_Ctrl_PID()
	{
		Kp = 1;
		Ki = 0;
		Kd = 2;
		kp_weight_amplify = 0.2;
		max_kp_weight = 2.5;
		kp_weight_diff_point = 1;
		  chang_pos_fuzzy=1.23;

		/****-----位置环模糊表格--------------------------------*/
		//偏差隶属度赋值
		int PFF_data[4] = { 0, 250, 500, 1000 };
		for (short i = 0; i < 4; i++)
		{
			pos_fuzzy_P.PFF[i] = PFF_data[i];
		}
		//变化隶属度赋值
		int DFF_data[4] = { 0, 200, 300, 400 };
		for (short i = 0; i < 4; i++)
		{
			pos_fuzzy_P.DFF[i] = DFF_data[i];
		}
		//输出隶属度赋值
		int UFF_data[7] = { 83, 100, 200, 250, 300, 350, 400 };
		for (short i = 0; i < 7; i++)
		{
			pos_fuzzy_P.UFF[i] = UFF_data[i];
		}
		//规则赋值
		int rule_data[7][7] = {
			//-3,-2,-1, 0, 1, 2, 3     // 误差
			{ 6, 6, 5, 1, 3, 3, 3, },   //   -3
			{ 6, 5, 4, 1, 3, 3, 3, },   //   -2
			{ 5, 4, 4, 1, 3, 3, 3, },   //   -1
			{ 3, 2, 1, 0, 1, 2, 3, },   //    0
			{ 3, 3, 3, 1, 4, 4, 5, },   //    1
			{ 3, 3, 3, 1, 4, 5, 6, },   //    2
			{ 3, 3, 3, 1, 5, 6, 6 } };  //    3
		for (short i = 0; i < 7; i++)
		{
			for (short j = 0; j < 7; j++)
			{
				pos_fuzzy_P.rule[i][j] = rule_data[i][j];
			}
		}
		/****-----位置环模糊表格解算--------------------------------*/
	}
	float chang_pos_fuzzy;
	Fuzzy_PID pos_fuzzy_P;
	float kp_weight_diff_point; //m
	float kp_weight_amplify;//kp放大系数
	float max_kp_weight;  //kp放大限幅
	float position_death_room;
	//pid系数
	float Kp;
	float Ki;
	float Kd;
	float err[3];
	//pid组合器
	float P;
	float Ii;
	float D;
	float pid_out;
	float pos_hold_pid(float T, float thr_weight, float aim_pos, float actual_pos, float actual_v);
};
struct   Speed_Ctrl_PID
{
	  Speed_Ctrl_PID()
	{
		Kp = 0.012;
		Ki = 0.005;
		Kd = 0.01;
		//初始化lqr系数
		//  0.1970   -1.4170    4.4605   -7.9790    8.7719   -5.9115    2.1975   -0.2426   -0.1070    0.0302
		//  1.0000   -5.8854   14.5174  -19.0150   13.4651   -4.0769   -0.5509    0.6143   -0.0427   -0.0260
		float num_data[10] = { 0.003376237179, -0.02286638764, 0.06450949645, -0.0930196781, 0.05887141442, 0.01844636704, -0.06302206986, 0.04898395296, -0.01794767665, 0.002668344203 };
		float den_data[10] = { 1.0, -5.024181794, 8.665231568, -2.729953284, -9.753097559, 12.1484876, -2.392381064, -4.526276674, 3.317787024, -0.7056158148 };

	//	float num_data[10] = { 0.0034 ,- 0.0229   , 0.0645, - 0.0930 ,   0.0589 ,   0.0184 ,- 0.0630  ,  0.0490, - 0.0179  ,  0.0027 };
	//	float den_data[10] = { 1.0000 ,- 5.0242 ,   8.6652 ,- 2.7300 ,- 9.7531  , 12.1485 ,- 2.3924, - 4.5263  ,  3.3178 ,- 0.7056 };
		for (int i = 0; i < 10; i++)
		{
			lqr_ctler.num[i] = num_data[i];
			lqr_ctler.den[i] = den_data[i];

		}
	}
	  //离散闭环传递函数控制
//	CTRL_Z_FUNCTION<10>  lqr_ctler;
	  CTRL_Z_FUNCTION2  lqr_ctler;
	//pid系数
	float Kp;
	float Ki;
	float Kd;
	float err[10];
	//pid组合器
	float P;
	float Ii;
	float D;
	float pid_out;
	float speed_hold_pid(float T, float thr_weight, float aim_speed, float actual_speed);
};
struct   Acc_HLOD_PID
{
	Acc_HLOD_PID()
	{
		Kp = 0.012;
		Ki = 0.005;
		Kd = 0.01;
	}
	//pid系数
	float Kp;
	float Ki;
	float Kd;
	float err[10];
	//pid组合器
	float P;
	float Ii;
	float D;
	float pid_out;
	float acc_hold_pid(float T, float thr_weight, float aim_acc, float actual_acc); 
};

class Pos_CTRL
{
public:
	
	Pos_CTRL()
	{
		Postion_hold_ctrl_on = 0;

		out_filter1 = 10;
		out_filter2 = 10; 
	}
	short Postion_hold_ctrl_on;
	short Pos_to_speed_on;

	Acc_HLOD_PID  x_acc_ctrl;
	Acc_HLOD_PID  y_acc_ctrl;
	Speed_Ctrl_PID x_v_ctrl;
	Speed_Ctrl_PID y_v_ctrl;
	POS_Ctrl_PID   x_pos_ctrl;
	POS_Ctrl_PID   y_pos_ctrl;
	Vector3f aim_position;
	Vector3f aim_position_b;//将目标点转化到机体系
	Vector3f out_speed;
	Vector3f aim_v;
	Vector3f aim_acc;
	Vector3f thr_aim_v;
	float out_pitch;
	float out_roll;
	float out_pit_fil1;
	float out_roll_fil1;
	float out_filter1;
	float out_filter2;
	void position_ctrl(float T,float thr_weight,float x_v_actual,float y_v_actual,float x_pos_actual,float y_pos_actual);
	void speed_ctrl(float T, float thr_weight, float x_acc_actual, float y_acc_actual, float x_v_actual, float y_v_actual);
	void acc_ctrl(float T, float thr_weight, float x_acc_actual, float y_acc_actual );
	void para_clear(void);


private:

};
extern Pos_CTRL pos_ctrl;


#endif