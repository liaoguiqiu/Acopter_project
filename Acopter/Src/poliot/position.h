#ifndef _POSITION_H
#define _POSITION_H

#include "gps.h"
#include "AHRS.h"

#include "imu.h"
struct  GPS_POSITION_HF
{

	GPS_POSITION_HF()
	{
		Death_room = 100;
		Ki = 0.005;
		I = 0;
	}
	float Death_room;
	float Ki;
	float I;
	float position_fl;
	float position_hf(float T, float pos, float speed);
};



class POSITION_H
{
public:
	POSITION_H()
	{

	}
	
	float xkf_px_b;
	float xkf_vn_b;
	float xkf_accx_b;
	float xkf_py_b;
	float xkf_ve_b;
	float xkf_accy_b;

	float xkf_px_n;
	float xkf_vn_n;
	float xkf_accx_n;
	float xkf_py_n;
	float xkf_ve_n;
	float xkf_accy_n;
	GPS_POSITION_HF x_position_hf;
	GPS_POSITION_HF y_position_hf;
	float x_pos_hf;
	float y_pos_hf;

  //flag
  bool pos_acc_mean_flag;

  short phase_lag_cnt;
  float wx_acc_ini;
  float wy_acc_ini;

  float wx_acc;
  float wx_acc_old[100];
  float wy_acc;
  float wy_acc_old[100];
  float acc_ki;
  Vector3f acc_i;
  Vector3f vec_old;
  //t����ϵ�µļ��ٶ�
  float pos_imu_accx;
  float pos_imu_accy;
  float pos_imu_accz;
  //��ʱ���ٶȶ���
  float pos_imu_accx_temp;
  float pos_imu_accy_temp;
  float pos_imu_accz_temp;
  
  //���ٶȼ���GPS�ٶ��ں�֮����ٶ�
  float pos_fuse_velo_x;
  float pos_fuse_velo_y;
  float pos_fuse_velo_z;
  
  //t����ϵ����GPS�õ���λ��
  float pos_gps_dis_x;
  float pos_gps_dis_y;
  float pos_gps_dis_z;
  
  float pos_gps_dis_x_old;
  float pos_gps_dis_y_old;
  float pos_gps_dis_z_old;
  
  //���ٶȼ���GPS�ٶ��ںϻ���֮���λ��
  float pos_fuse_displacement_x;
  float pos_fuse_displacement_y;
  float pos_fuse_displacement_z;
  
  //��GPS���ں��ٶȻ���λ�� �õ���λ��
  float pos_fuse_postion_x;
  float pos_fuse_postion_y;
  float pos_fuse_postion_z;
  
  //t����ϵ�µļ��ٶ�һ�λ���
  float pos_imu_acc_int1_x;
  float pos_imu_acc_int1_y;
  float pos_imu_acc_int1_z;
  
  //t����ϵ�µļ��ٶȶ��λ���
  float pos_imu_acc_int2_x;
  float pos_imu_acc_int2_y;
  float pos_imu_acc_int2_z;
  
  //gps�ٶȷֱ�ͶӰ������ı�����ϵ
  float pvx,pvy,pvz,pos_test;
  double pvsum;

  //gps�õ����ٶ�������
  float gps_vx,gps_vy,gps_vz;
  float gps_vx_temp,gps_vy_temp,gps_vz_temp;
  
  //gps�ٶȺͼ��ٶȼ�һ�λ��ֵõ����ٶȱ���ϵ��
  float kv1;
  float kdis1,kdis2;
  
  
  //����
  //void pos_data_pre();//�����ں��е�����׼��
  //void pos_fuse_velo();//�ٶ��ں�
  void pos_speed_displacement_with_gps(float T);//λ���ں�
  float pos_median_filter_x(float * para);
  float pos_median_filter_xyz_x(float * para);
  
  float pos_median_filter_y(float * para);
  float pos_median_filter_xyz_y(float * para);
  
  float pos_median_filter_z(float * para);
  float pos_median_filter_xyz_z(float * para);
  void position_hf_xy(float T);

  //
//  matrix::Matrix<float, 3, 1>   line_acc_err;
  void caculate_line_acc_err(void);
  void  trans_line_acc_err(float acc_x, float acc_y, float acc_z);
private:
  
  
};

extern POSITION_H position;


#endif