#ifndef __KALMAN_H
#define __KALMAN_H

 
 #include <stdio.h>
#include <iostream>
#include <matrix/math.hpp>


struct KALMAN_basic
{
	float prevData ;
	float p  , q , r  , kGain ;
	float kalmanFilter(float inData);
        KALMAN_basic()
        {
          prevData = 0;
	  p = 10, q = 0.001, r = 0.02, kGain = 0;
        
        }
        
	void matrix_test();
	
};

struct Pos_Acc_Kalman
{
	matrix::Matrix<float, 2, 2>  F;//过程矩阵
	matrix::Matrix<float, 2, 1>  B;//控制矩阵
	matrix::Matrix<float, 1, 2>  H;//观测矩阵
	matrix::Matrix<float, 2, 2>  P;//过程预测协方差矩阵
	matrix::Matrix<float, 1,1>  R;//过程噪声
	matrix::Matrix<float, 1, 1>  Q;//观测噪声
	matrix::SquareMatrix<float, 1>  S;// 

	matrix::Matrix<float, 2, 1>  X;//估计结果
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 2, 1>  K;//估计结果
	matrix::Matrix<float, 1, 1>  Z;//信息
	void pos_acc_kalman(float T, float pos, float acc);
	Pos_Acc_Kalman()
	{
		F.setIdentity();
		H(0, 0) = 1;	H(1, 0) = 0;
		Q(0, 0) =0.0009;
		R(0, 0) = 0.009;
		 
	}
};


struct Pos_WZ_Kalman
{
	//卡尔曼滤波参数与函数
	//float dt = 0.005;//注意：dt的取值为kalman滤波器采样时间
	float X, X_dot;//位置和速度
	float P[2][2];
	float Pdot[4];
	float Q_X  , Q_dot  ; //角度数据置信度,角速度数据置信度
	float R_X , C_0  ;
	float q_bias, X_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	short start_cnt;
	Pos_WZ_Kalman()
	{
		start_cnt = 0;
		Q_X = 2;
		Q_dot = 0.00005; //角度数据置信度,角速度数据置信度
		R_X = 50;
		C_0 = 1;
		P[0][0] = 1;
		P[0][1] = 0;
		P[1][0] = 0;
		P[1][1] = 1;
	}

	void Kalman_Filter(float dt, float X_m, float dot_m);//angleAx 和 dotGy;
};



#endif
