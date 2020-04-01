#ifndef GPS_KALMAN_X_V3_H
#define GPS_KALMAN_X_V3_H

#include <stdio.h>
#include <iostream>
#include <matrix/math.hpp>
#include "kalman.h"



struct GPS_KALMAN_X_V3_UPDATE
{
 
  GPS_KALMAN_X_V3_UPDATE()
  {
    //构造函数
    speed_kp=0.5; 
    speed_ki=0.01;
    acc_kp=0.0;
    acc_ki=0.0;
  }
  //变量
  bool set_flag;
  float last_pos;


  float   speed_0, speed_i, acc_i, acc_0, acc_old, speed_old;

  float acc_temp;
  float  acc_mms2;
  float error_pos;
  float error_speed;
  float error_acc;
  float int_speed;//速度修正积分项
  float speed_kp;
  float speed_ki;
  float acc_kp;
  float acc_ki;
  
  
  matrix::Matrix<float, 2, 2>  F;        //一步预测矩阵
  matrix::Matrix<float, 1, 2>  H;        //量测矩阵
  matrix::Matrix<float, 2, 2>  P0;       //
  matrix::Matrix<float, 2, 2>  P1;       //
  matrix::Matrix<float, 2, 1>  B;        //
  matrix::Matrix<float, 2, 1>  xn;       //
  matrix::Matrix<float, 2, 2>  EYE2;     //
  matrix::Matrix<float, 2, 1>  K;        //增益矩阵
  matrix::Matrix<float, 2, 1>  Xkf;       //
  matrix::SquareMatrix<float, 1>  temp1;// 
  
  matrix::Matrix<float, 1, 1>  U;        //加速度值
  matrix::Matrix<float, 1, 1>  Z;        //GPS位置值
  
  matrix::Matrix<float, 1, 1>  R;
  matrix::Matrix<float, 2, 2>  Q;
  
  float q1,q2;
  float r;
  //函数
  void  kalman_init(float T, float pos, float acc);
  void kalman_updata(float T, float gps_pos, float gps_v,float acc);
  
 
};

class GPS_KALMAN_V3
{
public:
	GPS_KALMAN_V3()
	{

		x_kal_fill.Q(0, 0) = 0.01; x_kal_fill.Q(1, 1) = 0.01;
		x_kal_fill.R(0, 0) = 0.0001;
		y_kal_fill.Q(0, 0) = x_kal_fill.Q(0, 0);
		y_kal_fill.Q(1, 1) = x_kal_fill.Q(1, 1);
		y_kal_fill.R(0, 0) = x_kal_fill.R(0, 0);
	}
	GPS_KALMAN_X_V3_UPDATE x_kalman_update;
	GPS_KALMAN_X_V3_UPDATE y_kalman_update;
	Pos_Acc_Kalman x_kal_fill;
	Pos_Acc_Kalman y_kal_fill;
	float vx_old[10];
	float vy_old[10];
	float acc_x_feed_back;
	float acc_y_feed_back;
        float acc_x_af;
        float acc_y_af;
        double inbuf_x[51];
        int index_x,i_x;//index用于更新输入序列  i用于计算序列，j用于指示系数
        double inbuf_y[51];
        int index_y,i_y;//index用于更新输入序列  i用于计算序列，j用于指示系数
	void Gps_kalman_v3_xy_update(float T);
        void Gps_kalman_v3_accx_filter(float * in, float * out);
        void Gps_kalman_v3_accy_filter(float * in, float * out);
private:

};
extern GPS_KALMAN_V3 gps_kalman_v3_update;
 
#endif