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
/************************************************************************/
/* gps水平位置模型                                                                     */
/************************************************************************/
struct GPS_POS_MAKOF_KF
{
	GPS_POS_MAKOF_KF()
	{
		A.setIdentity();
		P.setIdentity();
		//B.setZero();
		//P(0, 0) = 10.00f; P(1, 1) = 10.0f; P(2, 2) = 0.01f;
		//P = P ;
		Q.setZero();
		R.setZero();
		H(0, 0) = 1.0f; H(0, 1) = 0; H(0, 2) = 1;
		H(1, 0) = 0; H(1, 1) = 1; H(1, 2) = 0;

		Q(0, 0) = 0.01f; Q(1, 1) = 1; Q(2, 2) = 0.0001f;
		R(0, 0) = 50; R(1, 1) = 100;
		  omega=1.005;//衰减因子
		  FORGET=0;//衰减开关
	}
	matrix::SquareMatrix<float, 3>  A;//过程矩阵
	//matrix::Matrix<float, 3, 1>  B;//控制矩阵
	matrix::Matrix<float, 2, 3>  H;//观测矩阵
	matrix::SquareMatrix<float, 1>  u;//控制过程
	matrix::SquareMatrix<float, 3>   P;//过程预测协方差矩阵
	matrix::SquareMatrix<float, 2>  R;//过程噪声
	matrix::SquareMatrix<float, 3>  Q;//观测噪声
	matrix::SquareMatrix<float, 2> save_i;

	matrix::Matrix<float, 3, 1>  X;//状态 位置，速度，漂移
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 3, 2>  K;//真有
	matrix::Matrix<float, 2, 1>   Z;//信息
	float omega;//衰减因子
	unsigned char FORGET;//衰减开关
	void gps_makof_KF(float T, float pos ,float speed);



};


/************************************************************************/
/* 两个位置与加表融合模型，只有一个漂移估计                                                                     */
/************************************************************************/
struct GPS_BARO_ACC_KF
{
	GPS_BARO_ACC_KF()
	{
		A.setIdentity();
		P.setIdentity();
		//B.setZero();
		//P(0, 0) = 10.00f; P(1, 1) = 10.0f; P(2, 2) = 0.01f;
		//P = P ;
		Q.setZero();
		R.setZero();
		H(0, 0) = 1.0f; H(0, 1) = 0; H(0, 2) = 1;
		H(1, 0) = 1.0f; H(1, 1) = 0; H(1, 2) = 0;

		Q(0, 0) = 0.1f; Q(1, 1) = 0.01f; Q(2, 2) = 0.2f;
		R(0, 0) = 60; R(1, 1) = 1000;
	}
	matrix::SquareMatrix<float, 3>  A;//过程矩阵
	matrix::Matrix<float, 3, 1>  B;//控制矩阵
	matrix::Matrix<float, 2, 3>  H;//观测矩阵
	matrix::SquareMatrix<float, 1>  u;//控制过程
	matrix::SquareMatrix<float, 3>   P;//过程预测协方差矩阵
	matrix::SquareMatrix<float, 2>  R;//过程噪声
	matrix::SquareMatrix<float, 3>  Q;//观测噪声
	matrix::SquareMatrix<float, 2> save_i;

	matrix::Matrix<float, 3, 1>  X;//状态 位置，速度，漂移
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 3, 2>  K;//真有
	matrix::Matrix<float, 2,1>   Z;//信息
	void gps_baro_acc_KF(float T, float pos1,float pos2, float acc);
	


};


/************************************************************************/
/* 位置加速度（矩阵）                                                                     */
/************************************************************************/
struct Pos_Acc_Kalman
{
	matrix::SquareMatrix<float, 2>  A;//过程矩阵
	matrix::Matrix<float, 2, 1>  B;//控制矩阵
	matrix::Matrix<float, 1, 2>  H;//观测矩阵
	matrix::SquareMatrix<float, 1>  u;//控制过程
	matrix::SquareMatrix<float, 2>   P;//过程预测协方差矩阵
	matrix::SquareMatrix<float, 1>  R;//过程噪声
	matrix::SquareMatrix<float, 2>  Q;//观测噪声


	matrix::Matrix<float, 2, 1>  X;//估计结果
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 2, 1>  K;//真有
	matrix::SquareMatrix<float, 1>   Z;//信息
	void pos_acc_kalman(float T, float pos, float acc);
	Pos_Acc_Kalman()
	{
		A.setIdentity();
		P.setIdentity();
		Q.setIdentity();
		R.setIdentity();
		H(0, 0) = 1;	H(0, 1) = 0;

		Q(0, 0) = 0.001; Q(1, 1) = 0.001;
		R(0, 0) =300;

	}
};



/************************************************************************/
/* 位置速度模型（矩阵）                                                                     */
/************************************************************************/
struct Pos_WZ_Kalman_Marix
{
	matrix::SquareMatrix<float, 2>  A;//过程矩阵
	matrix::Matrix<float, 2, 1>  B;//控制矩阵
	matrix::SquareMatrix<float, 2>  H;//观测矩阵
	matrix::SquareMatrix<float, 1>  u;//控制过程
	matrix::SquareMatrix<float, 2>   P;//过程预测协方差矩阵
	matrix::SquareMatrix<float, 2>  R;//过程噪声
	matrix::SquareMatrix<float, 2>  Q;//观测噪声


	matrix::Matrix<float, 2, 1>  X;//估计结果
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::SquareMatrix<float, 2> K;//真有
	matrix::Matrix<float, 2, 1>    Z;//信息
	void pos_wz_kalman(float T, float pos, float v, float acc);
	Pos_WZ_Kalman_Marix()
	{
		A.setIdentity();
		P.setIdentity();
		Q.setIdentity();
		R.setIdentity();
		H(0, 0) = 1;	H(0, 1) = 1;

		Q(0, 0) = 0.3; Q(1, 1) = 0.015;
		R(0, 0) = 600; R(1, 1) = 300;

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
		Q_dot = 0.005; //角度数据置信度,角速度数据置信度
		R_X = 500;
		C_0 = 100;
		P[0][0] = 1;
		P[0][1] = 0;
		P[1][0] = 0;
		P[1][1] = 1;
	}

	void Kalman_Filter(float dt, float X_m, float dot_m);//angleAx 和 dotGy;
};



#endif
