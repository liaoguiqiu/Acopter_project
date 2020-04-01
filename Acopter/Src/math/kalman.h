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
/* gpsˮƽλ��ģ��                                                                     */
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
		  omega=1.005;//˥������
		  FORGET=0;//˥������
	}
	matrix::SquareMatrix<float, 3>  A;//���̾���
	//matrix::Matrix<float, 3, 1>  B;//���ƾ���
	matrix::Matrix<float, 2, 3>  H;//�۲����
	matrix::SquareMatrix<float, 1>  u;//���ƹ���
	matrix::SquareMatrix<float, 3>   P;//����Ԥ��Э�������
	matrix::SquareMatrix<float, 2>  R;//��������
	matrix::SquareMatrix<float, 3>  Q;//�۲�����
	matrix::SquareMatrix<float, 2> save_i;

	matrix::Matrix<float, 3, 1>  X;//״̬ λ�ã��ٶȣ�Ư��
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 3, 2>  K;//����
	matrix::Matrix<float, 2, 1>   Z;//��Ϣ
	float omega;//˥������
	unsigned char FORGET;//˥������
	void gps_makof_KF(float T, float pos ,float speed);



};


/************************************************************************/
/* ����λ����ӱ��ں�ģ�ͣ�ֻ��һ��Ư�ƹ���                                                                     */
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
	matrix::SquareMatrix<float, 3>  A;//���̾���
	matrix::Matrix<float, 3, 1>  B;//���ƾ���
	matrix::Matrix<float, 2, 3>  H;//�۲����
	matrix::SquareMatrix<float, 1>  u;//���ƹ���
	matrix::SquareMatrix<float, 3>   P;//����Ԥ��Э�������
	matrix::SquareMatrix<float, 2>  R;//��������
	matrix::SquareMatrix<float, 3>  Q;//�۲�����
	matrix::SquareMatrix<float, 2> save_i;

	matrix::Matrix<float, 3, 1>  X;//״̬ λ�ã��ٶȣ�Ư��
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 3, 2>  K;//����
	matrix::Matrix<float, 2,1>   Z;//��Ϣ
	void gps_baro_acc_KF(float T, float pos1,float pos2, float acc);
	


};


/************************************************************************/
/* λ�ü��ٶȣ�����                                                                     */
/************************************************************************/
struct Pos_Acc_Kalman
{
	matrix::SquareMatrix<float, 2>  A;//���̾���
	matrix::Matrix<float, 2, 1>  B;//���ƾ���
	matrix::Matrix<float, 1, 2>  H;//�۲����
	matrix::SquareMatrix<float, 1>  u;//���ƹ���
	matrix::SquareMatrix<float, 2>   P;//����Ԥ��Э�������
	matrix::SquareMatrix<float, 1>  R;//��������
	matrix::SquareMatrix<float, 2>  Q;//�۲�����


	matrix::Matrix<float, 2, 1>  X;//���ƽ��
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::Matrix<float, 2, 1>  K;//����
	matrix::SquareMatrix<float, 1>   Z;//��Ϣ
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
/* λ���ٶ�ģ�ͣ�����                                                                     */
/************************************************************************/
struct Pos_WZ_Kalman_Marix
{
	matrix::SquareMatrix<float, 2>  A;//���̾���
	matrix::Matrix<float, 2, 1>  B;//���ƾ���
	matrix::SquareMatrix<float, 2>  H;//�۲����
	matrix::SquareMatrix<float, 1>  u;//���ƹ���
	matrix::SquareMatrix<float, 2>   P;//����Ԥ��Э�������
	matrix::SquareMatrix<float, 2>  R;//��������
	matrix::SquareMatrix<float, 2>  Q;//�۲�����


	matrix::Matrix<float, 2, 1>  X;//���ƽ��
	matrix::Matrix<float, 1, 1>  V;// 
	matrix::SquareMatrix<float, 2> K;//����
	matrix::Matrix<float, 2, 1>    Z;//��Ϣ
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
		Q_dot = 0.005; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�
		R_X = 500;
		C_0 = 100;
		P[0][0] = 1;
		P[0][1] = 0;
		P[1][0] = 0;
		P[1][1] = 1;
	}

	void Kalman_Filter(float dt, float X_m, float dot_m);//angleAx �� dotGy;
};



#endif
