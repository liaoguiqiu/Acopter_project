/************************************************************************/
/* 化为反馈式的LQR状态反馈
copy right  ： Guiqiu LIao */
/************************************************************************/

#ifndef _LQR_FEED_H
#define _LQR_FEED_H
#include "mymath.h"

#define  SIMULINK_ON 0



template<int LEN>
class CTRL_Z_FUNCTION
{
public:
	CTRL_Z_FUNCTION()
	{
		out = 0;
		thi_limit = 1000000000;// limit
	}
	float in_old[LEN]; 
	float out_old[LEN];
	float num[LEN];
	float den[LEN];
	float out;
	float thi_limit;
	float  z_function_discreate_ctrl(float in)
	{
		//输入的保存
		for (int i = 1; i < LEN; i++)
		{
			in_old[LEN - i] = in_old[LEN - i - 1];
		}
		in_old[0] = in;
		//输出的保存
		for (int i = 1; i < LEN; i++)
		{
			out_old[LEN - i] = out_old[LEN - i - 1];
		}
		//更新输出
		out = 0;
		for (int i = 1; i < LEN; i++)
		{
			out = out - den[i] * out_old[i];
		}
		for (int i = 0; i < LEN; i++)
		{
			out = out + num[i] * in_old[i];
		}
	//	out = LIMIT(out, -thi_limit, thi_limit);//limi
		out_old[0] = out;
		return out;
	}
};
 

struct CTRL_Z_FUNCTION2
{
	CTRL_Z_FUNCTION2()
	{
		out = 0;
		thi_limit = 1000000000;// limit
	}
	float in_old[10];
	float out_old[10];
	float num[10];
	float den[10];
	float out;
	float thi_limit;
	float  z_function_discreate_ctrl(float in);
};
class CTRL_SIMULINK
{
public:
	CTRL_SIMULINK()
	{
		float num_speed_data[4] = {0, 0.02998101973, 0.1179278857, 0.02935788749};
		float den_speed_data[4] = { 1.0, -2.836930401, 2.795878143, -0.9589477418 };
		for (int i = 0; i < 4;i++)
		{
			sys_speed.num[i] = num_speed_data[i];
			sys_speed.den[i] = den_speed_data[i];
		}


	}
	CTRL_Z_FUNCTION<4>  sys_speed; //速度为四阶系统
	float aim_pos;
	float real_pos;
	float aim_speed;
	float real_speed;
	float simu_speed_ctrl(float T);
	float simu_pos_ctrl(float T);

};
extern CTRL_SIMULINK simulink;

#endif
