#ifndef __IMU_KALMAN_H
#define __IMU_KALMAN_H


#include <stdio.h>
#include <iostream>
#include <matrix/math.hpp>
#include "mymath.h"
class IMU_EXTEND_KALMAN
{
public:
	IMU_EXTEND_KALMAN()
	{
		gravity_mps2 = 9.81;
		mag3D_unitVector_meas = 0.3;
		accel_meas_mps2 = 0.5;
		gyro_bias_rps = 0.005;
		Quaternion_process_noise = 0.00004;
		gyroBias_process_noise_rps = 0.0001;
		  xhat_k.setZero();
		  xhat_k(0, 0) = 1;
		  P.setIdentity();
		  Q.setIdentity();
		  Q = Q*Quaternion_process_noise*Quaternion_process_noise;
		  Q(4, 4) = gyroBias_process_noise_rps*gyroBias_process_noise_rps;
		  Q(5, 5) = gyroBias_process_noise_rps*gyroBias_process_noise_rps;
		  Q(6, 6) = gyroBias_process_noise_rps*gyroBias_process_noise_rps;

		  R.setIdentity();
		  R = R*accel_meas_mps2*accel_meas_mps2;
		  R(3, 3) = mag3D_unitVector_meas*mag3D_unitVector_meas;
		  R(4, 4) = mag3D_unitVector_meas*mag3D_unitVector_meas;
		  R(5, 5) = mag3D_unitVector_meas*mag3D_unitVector_meas;

	}
	matrix::Matrix<float, 7, 1>  xhat_k;// 
	matrix::Matrix<float, 6, 1> z_k;
	matrix::Matrix<float, 7, 6>  K;// 
	matrix::SquareMatrix<float, 7>  P;//
	matrix::SquareMatrix<float, 7>  Q;// 
	matrix::SquareMatrix<float, 6>  R;//
	matrix::Matrix<float, 3, 1>  u1;// 
	matrix::Matrix<float, 3, 1>  u2;// 
	float  u3;// 重力向量
	float  u4;// imu安装误差（磁力计）
	matrix::SquareMatrix<float, 3> C_ned2b_save;//姿态余弦矩阵
	matrix::SquareMatrix<float, 3> C_ned2b_body;//体系，没有考虑航向角
	float gravity_mps2  ;
	float mag3D_unitVector_meas;
	float accel_meas_mps2;
	float gyro_bias_rps;
	float Quaternion_process_noise;
	float gyroBias_process_noise_rps;
	float roll;
	float pith;
	float yaw;
	float roll_rad;
	float pith_rad;
	float yaw_rad;
	float test_vx;
	float test_vy;
	float yaw_MAG;
	float yaw_MAG_rad;
	matrix::Matrix<float, 3, 1> mag3D_measure_in_body;
	void Imu_emistate_altitude_ekf(float T, float gx, float gy, float gz, float ax, float ay, float az,
		float mx, float my, float mz, float vx, float vy, float vz);

	void Imu_ekf(float T,float gx, float gy, float gz, float ax, float ay, float az,
		         float mx,float my , float mz,float vx,float vy,  float vz);
	void Imu_ekf_q4_to_euler();

private:

};
extern IMU_EXTEND_KALMAN  imu_ekf;

#endif