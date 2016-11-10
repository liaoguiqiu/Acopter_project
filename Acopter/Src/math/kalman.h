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
	matrix::Matrix<float, 2, 2>  F;//���̾���
	matrix::Matrix<float, 2, 1>  B;//���ƾ���
	matrix::Matrix<float, 1, 2>  H;//�۲����
	matrix::Matrix<float, 2, 2>  P;//����Ԥ��Э�������
	matrix::Matrix<float, 1,1>  R;//��������
	matrix::Matrix<float, 1, 1>  Q;//�۲�����
	matrix::SquareMatrix<float, 1>  S;// 

	matrix::Matrix<float, 2, 1>  X;//���ƽ��
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 2, 1>  K;//���ƽ��
	matrix::Matrix<float, 1, 1>  Z;//��Ϣ
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
	//�������˲������뺯��
	//float dt = 0.005;//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��
	float X, X_dot;//λ�ú��ٶ�
	float P[2][2];
	float Pdot[4];
	float Q_X  , Q_dot  ; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�
	float R_X , C_0  ;
	float q_bias, X_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	short start_cnt;
	Pos_WZ_Kalman()
	{
		start_cnt = 0;
		Q_X = 2;
		Q_dot = 0.00005; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�
		R_X = 50;
		C_0 = 1;
		P[0][0] = 1;
		P[0][1] = 0;
		P[1][0] = 0;
		P[1][1] = 1;
	}

	void Kalman_Filter(float dt, float X_m, float dot_m);//angleAx �� dotGy;
};



#endif
