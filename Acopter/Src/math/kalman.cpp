#include "kalman.h"
#include "mymath.h"
// 
//void KALMAN_basic:: matrix_test()
//{
//	float data[9] = { 1, 0, 0, 0, 1, 0, 1, 0, 1 };
//	matrix:: Matrix3f A(data);
//	float data_check[9] = { 1, 0, 0, 0, 1, 0, -1, 0, 1 };
//	matrix::Matrix3f A_I(data_check);
//	matrix::Matrix2f I;
//	I.setIdentity();
//	matrix::Matrix3f R = A * A_I;
//	R = A.I();//求逆矩阵
//	R = A.transpose();//转AT
//	float data1[] = { 1, 2};
//	float data2[] = { 6, 7};
//	matrix::Vector<float, 2> v1(data1);
//	v1 = I* v1;
//	matrix::Matrix<float, 2, 1> v2;
//	v2 = v1;
//	matrix::Matrix<float, 1, 2> v3;
//	v3 = v2.transpose();
//
//	v3 = v3*I;
//	 
//}

#define  NEED_FORGET 1
#define  THRESHOLD_ON_FORGET    1000   //mm
void GPS_POS_MAKOF_KF:: gps_makof_KF(float T, float pos, float speed)
{

	/*-----------*/
	//B(0, 0) = 0.5f*T*T;
	//B(1, 0) = T;
	//B(2, 0) = 0;
	A(0, 1) = T;
//	u(0, 0) = acc;
	Z(0, 0) = pos;
	Z(1, 0) = speed;

	//预测
	X = A*X;   //+ B*u;
	//对误差矩阵进行衰减记忆
#if NEED_FORGET
	float abs_err = ABS(X(0, 0) - pos);
	if (abs_err>THRESHOLD_ON_FORGET)
	{
		FORGET = 1;
	}
	else// if (abs_err<20)
	{
		FORGET=0;
	}

	if (FORGET==1)
	{
		P = A*(P*omega)*A.transpose() + Q;
	} 
	else
	{
		P = A*P*A.transpose() + Q;
	}


#else
	P = A*P*A.transpose() + Q;
#endif // DEBUG

	//校准

	save_i = H*P*H.transpose() + R;
	K = P*H.transpose()*save_i.I();
	X = X + K*(Z - H*X);
	P = P - K*H*P;



}

void GPS_BARO_ACC_KF::gps_baro_acc_KF(float T, float pos1, float pos2, float acc)
{

	/*-----------*/
	B(0, 0) = 0.5f*T*T;
	B(1, 0) = T;
	B(2, 0) = 0;
	A(0, 1) = T;
	u(0, 0) = acc;
	Z(0, 0) = pos1;
	Z(1, 0) = pos2;

	//预测
	X = A*X + B*u;
	P = A*P*A.transpose() + Q;

	//校准
	
	save_i = H*P*H.transpose() + R;
	K = P*H.transpose()*save_i.I();
	X = X + K*(Z - H*X);
	P = P - K*H*P;



}


float KALMAN_basic::kalmanFilter(float inData)
{



	p = p + q;
	kGain = p / (p + r);

	inData = prevData + (kGain*(inData - prevData));
	p = (1 - kGain)*p;

	prevData = inData;

	return inData;

}
 
/************************************************************************/
/* 位置速度模型（矩阵）                                                                     */
/************************************************************************/
void Pos_WZ_Kalman_Marix::pos_wz_kalman(float T, float pos, float v, float acc)
{
	/*-----------*/
	B(0, 0) = 0.5f*T*T;
	B(1, 0) = T;
	A(0, 1) = T;
	u(0, 0) = acc;
	Z(0, 0) = pos;
	Z(1, 0) = v;
	//预测
	X = A*X + B*u;
	P = A*P*A.transpose() + Q;

	//校准
	matrix::SquareMatrix<float, 2> save_i;
	save_i = H*P*H.transpose() + R;
	K = P*H.transpose()*save_i.I();
	X = X + K*(Z - H*X);
	P = P - K*H*P;
}

void Pos_Acc_Kalman::pos_acc_kalman(float T, float pos, float acc)
{
	/*-----------*/
	B(0, 0) = 0.5f*T*T;
	B(1, 0) = T;
	A(0, 1) = T;
	u(0, 0) = acc;
	Z(0, 0) = pos;

	//预测
	X = A*X + B*u;
	P = A*P*A.transpose() + Q;

	//校准
	matrix::SquareMatrix<float, 1> save_i;
	save_i = H*P*H.transpose() + R;
	K = P*H.transpose()*save_i.I();
	X = X + K*(Z - H*X);
	P = P - K*H*P;

}



//卡尔曼滤波
void Pos_WZ_Kalman::Kalman_Filter(float dt, float X_m, float dot_m)//XAx 和 dotGy
{
	X += (dot_m - q_bias) * dt;
	X_err = X_m - X;
	Pdot[0] = Q_X - P[0][1] - P[1][0];
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_dot;
	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	E = R_X + C_0 * PCt_0;
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];
	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	X += K_0 * X_err; //最优角度
	q_bias += K_1 * X_err;
	X_dot = dot_m - q_bias;//最优角速度
}
