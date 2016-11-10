#include "kalman.h"
 
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



float KALMAN_basic::kalmanFilter(float inData)
{



	p = p + q;
	kGain = p / (p + r);

	inData = prevData + (kGain*(inData - prevData));
	p = (1 - kGain)*p;

	prevData = inData;

	return inData;

}
void Pos_Acc_Kalman:: pos_acc_kalman(float T, float pos, float acc)
{
/*-----------*/
	B(0, 0) = 0.5f*T*T;
	B(1, 0) = T;
	F(0, 1) = T;
	P = F*P*F.transpose() + (B*Q*B.transpose());
	Z(0,0) = pos;		 
	S = (H*P*H.transpose() + R);
	S = S.I();
    K =( P*H.transpose() )*S;
	X = F*X + B*acc;
	
	V = Z - H*X;
	X = X + K*V;


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
