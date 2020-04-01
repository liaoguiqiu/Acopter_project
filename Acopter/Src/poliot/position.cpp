#include "position.h"
#include "height_ctrl.h"
#include "Butter.h"
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include "arm_math.h"
#include "mymath.h"
#include "gps_kalman.h"
#include "gps.h"
#include "gps_kalman_v3.h"
#include "baro.h"
#include "imu_ekf2.h"
#define POS_IMU_INT1 7
#define POS_IMU_INT2 9
#define POS_FUSE_INT1 5
#define  USE_DEVER_INTERGRAL 1  //使用分段积分进行互补


POSITION_H position;
Butter2 butter_pos_x;
Butter2 butter_pos_y;
Butter2 butter_pos_z;

/************************************************************************/
/* 位置点的高通滤波器                                                                     */
/************************************************************************/

float GPS_POSITION_HF:: position_hf(float T, float pos, float speed)
{

/************************************************************************/
/* 分段积分打开                                                                     */
/************************************************************************/
#if  USE_DEVER_INTERGRAL
	float pos_err;
	pos_err = (pos - position_fl);
	if (ABS(pos_err) < 4000)
	{
		I = Ki*pos_err*T;
	}
	else
	{
		I = 10 * Ki*pos_err*T;
	}

#else
	I = Ki*(pos - position_fl)*T;
#endif

	
	position_fl += (my_deathzoom(speed, Death_room)*T + I);
	
	return position_fl;
}

void POSITION_H::position_hf_xy( float T)
{

	 
	x_pos_hf = x_position_hf.position_hf(T, gps.gps_pos_x_b*1000.0f, gps.gps_v_x_b*1000.0f );
	y_pos_hf = y_position_hf.position_hf(T, gps.gps_pos_y_b*1000.0f, gps.gps_v_y_b*1000.0f );

}
void POSITION_H::pos_speed_displacement_with_gps(float T)
{ 
   
	//float wx_acc_n,wy_acc_n,wx_acc_b,wy_acc_b;
	//wx_acc_n = (imu_ekf2.C_ned2b_save(2, 0) *ahrs.Acc.z + imu_ekf2.C_ned2b_save(0, 0) * ahrs.Acc.x + imu_ekf2.C_ned2b_save(1, 0)* ahrs.Acc.y) / 8200 * 9810;
	//wy_acc_n = (imu_ekf2.C_ned2b_save(2, 1) *ahrs.Acc.z + imu_ekf2.C_ned2b_save(0, 1) * ahrs.Acc.x + imu_ekf2.C_ned2b_save(1, 1)* ahrs.Acc.y) / 8200 * 9810;
	//wx_acc_b = wx_acc_n *cos(imu_ekf2.yaw*M_PI_F / 180.0f) + wy_acc_n *sin(imu_ekf2.yaw*M_PI_F / 180.0f);
	//wy_acc_b = -wx_acc_n *sin(imu_ekf2.yaw*M_PI_F / 180.0f) + wy_acc_n *cos(imu_ekf2.yaw*M_PI_F / 180.0f);
	//wx_acc += (1 / (1 + 1 / (2 * 3.14f *T))) *(wx_acc_b - wx_acc);
	//wy_acc += (1 / (1 + 1 / (2 * 3.14f *T))) *(wy_acc_b - wy_acc);

	//时间延迟
	/*phase_lag_cnt++;
	if (phase_lag_cnt>=30)
	{
		phase_lag_cnt = 0;
		wx_acc_old[0] = wx_acc;
		wy_acc_old[0] = wy_acc;
	}*/

	for (int i = 99; i > 0; i--)
	{
		wx_acc_old[i] = wx_acc_old[i - 1];
		wy_acc_old[i] = wy_acc_old[i - 1];
	}
	wx_acc_old[0] = wx_acc;
	wy_acc_old[0] = wy_acc;
	wx_acc_ini = (imu_dcm.a_x*ahrs.Acc.x + imu_dcm.a_y*ahrs.Acc.y + imu_dcm.a_z*ahrs.Acc.z) / 8200 * 9810;
	wy_acc_ini = (imu_dcm.b_x*ahrs.Acc.x + imu_dcm.b_y*ahrs.Acc.y + imu_dcm.b_z*ahrs.Acc.z) / 8200 * 9810;
  wx_acc += (1 / (1 + 1 / (5.0f *3.14f *T))) *( wx_acc_ini- wx_acc);
  wy_acc += (1 / (1 + 1 / (5.0f *3.14f *T))) *( wy_acc_ini- wy_acc);
 
  //wx_acc = (imu_dcm.a_x*ahrs.Acc.x + imu_dcm.a_y*ahrs.Acc.y + imu_dcm.a_z*ahrs.Acc.z)/8200*9800;
  //wy_acc = (imu_dcm.b_x*ahrs.Acc.x + imu_dcm.b_y*ahrs.Acc.y + imu_dcm.b_z*ahrs.Acc.z)/8200*9800;
  //机体系
  xkf_px_b = (float)gps_kalman_v3_update.x_kalman_update.Xkf(0, 0);
  xkf_vn_b = (float)gps_kalman_v3_update.x_kalman_update.Xkf(1, 0);
  xkf_py_b = (float)gps_kalman_v3_update.y_kalman_update.Xkf(0, 0);
  xkf_ve_b = (float)gps_kalman_v3_update.y_kalman_update.Xkf(1, 0);
  //导航系
  xkf_px_n = xkf_px_b*cos(imu_ekf2.yaw*M_PI_F / 180.0f) - xkf_py_b*sin(imu_ekf2.yaw*M_PI_F / 180.0f);
  xkf_vn_n = xkf_vn_b*cos(imu_ekf2.yaw*M_PI_F / 180.0f) - xkf_ve_b*sin(imu_ekf2.yaw*M_PI_F / 180.0f);
  xkf_py_n = xkf_px_b*sin(imu_ekf2.yaw*M_PI_F / 180.0f) + xkf_py_b*cos(imu_ekf2.yaw*M_PI_F / 180.0f);
  xkf_ve_n = xkf_vn_b*sin(imu_ekf2.yaw*M_PI_F / 180.0f) + xkf_ve_b*cos(imu_ekf2.yaw*M_PI_F / 180.0f);
}
 
void POSITION_H:: caculate_line_acc_err(void)
{


	//trans_line_acc_err(0, 0, baro.acc_z_kf);
	trans_line_acc_err(gps_kalman_v3_update.acc_x_feed_back, (-gps_kalman_v3_update.acc_x_feed_back), baro.acc_z_feed_back);
	//trans_line_acc_err(0.5*wx_acc_ini, 0.5*wy_acc_ini, baro.acc_z_feed_back);

}

 void POSITION_H::trans_line_acc_err(float acc_x, float acc_y, float acc_z)
{
	float line_acc_err_data[3] = { acc_x, acc_y, acc_z };
	 matrix::Matrix<float, 3, 1> acc_m(line_acc_err_data);
	 /*
	matrix::SquareMatrix<float, 3>  trans_DCM = imu_ekf.C_ned2b_save.transpose();
	line_acc_err = trans_DCM.I() * acc_m*8200/9810;*/

//	line_acc_err = imu_ekf.C_ned2b_body.I() * acc_m * 8200 / 9810; 
}