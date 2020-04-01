/************************************************************************/
/* ��Ϊ����ʽ��LQR״̬����
copy right  �� Guiqiu LIao */
/************************************************************************/

#include "LQR_ctrl.hpp"
#include "pos_ctrl.h"
CTRL_SIMULINK simulink;
//
//float z_function_discreate_ctrl(float in)
//{
//	//����ı���
//	for (int i = 1; i < LEN; i++)
//	{
//		in_old[LEN - i] = in_old[LEN - i - 1];
//	}
//	in_old[0] = in;
//	//����ı���
//	for (int i = 1; i < LEN; i++)
//	{
//		out_old[LEN - i] = out_old[LEN - i - 1];
//	}
//	//�������
//	out = 0;
//	for (int i = 1; i < LEN; i++)
//	{
//		out = out - den[i] * out_old[i];
//	}
//	for (int i = 0; i < LEN; i++)
//	{
//		out = out + num[i] * in_old[i];
//	}
//	out = LIMIT(out, -thi_limit, thi_limit);//limi
//	out_old[0] = out;
//
//}
float CTRL_Z_FUNCTION2:: z_function_discreate_ctrl(float in)
{
	int LEN = 10;
	//����ı���
	for (int i = 1; i < LEN; i++)
	{
		in_old[LEN - i] = in_old[LEN - i - 1];
	}
	in_old[0] = in;
	//����ı���
	for (int i = 1; i < LEN; i++)
	{
		out_old[LEN - i] = out_old[LEN - i - 1];
	}
	//�������
	out = 0;
	for (int i = 1; i < LEN; i++)
	{
		out = out - den[i] * out_old[i];
	}
	for (int i = 0; i < LEN; i++)
	{
		out = out + num[i] * in_old[i];
	}
	out = LIMIT(out, -thi_limit, thi_limit);//limi
	out_old[0] = out;

}

float CTRL_SIMULINK::simu_speed_ctrl(float T)
{
	//������
	aim_speed = pos_ctrl.x_pos_ctrl.pid_out;
	pos_ctrl.x_v_ctrl.speed_hold_pid(T, 1, aim_speed, real_speed);
	//ϵͳ��
	real_speed = sys_speed.z_function_discreate_ctrl(pos_ctrl.x_v_ctrl.pid_out*0.001);
	real_pos = real_pos + real_speed *T;

}
float CTRL_SIMULINK::simu_pos_ctrl(float T)
{
	pos_ctrl.x_pos_ctrl.pos_hold_pid(T, 1, aim_pos, real_pos, real_speed);

}