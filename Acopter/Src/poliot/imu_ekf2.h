#ifndef __IMU_EKF_H
#define __IMU_EKF_H


#include <stdio.h>
#include <iostream>
#include <matrix/math.hpp>
#include "mymath.h"


typedef struct
{
	Vector3f g;
	Vector3f a;
	float Yaw_mag_old;
	float delta_T;

}  INS_DATA_BUFF_SAVE;


class IMU_EKF2
{
public:
	IMU_EKF2()
	{

		Filter_ON = 0;
		X_GPS_DISTANCE = 0.065; // m
		Z_GPS_Height =0.085; // m
		INS_DELAY_TIME = 20;
		gravity_mps2 = 9.81;
		xhat_k.setZero();
		xhat_k(0, 0) = 1;
	


		Quaternion_process_noise = 0.00001f;
		pos_process_noise_m = 0.0000001f;
		vel_process_noise_mps = 0.00001f;
		gyroBias_process_noise_rps = 0.00001f;
		accelBias_process_noise_mps2 = 0.000001f;
		ekfQ.setZero();
		//看来初始化时大运算矩阵承受不了
		ekfQ(0,0) =  Quaternion_process_noise*Quaternion_process_noise;
		ekfQ(1, 1) = ekfQ(0, 0);
		ekfQ(2, 2) = ekfQ(0, 0);
		ekfQ(3, 3) = ekfQ(0, 0);
		ekfQ(4, 4) = vel_process_noise_mps*vel_process_noise_mps;
		ekfQ(5, 5) = ekfQ(4, 4);
		ekfQ(6, 6) = ekfQ(4, 4);
		ekfQ(7, 7) = gyroBias_process_noise_rps*gyroBias_process_noise_rps;
		ekfQ(8, 8) = ekfQ(7, 7);
		ekfQ(9, 9) = ekfQ(7, 7);
		ekfQ(10, 10) = accelBias_process_noise_mps2*accelBias_process_noise_mps2;
		ekfQ(11, 11) = ekfQ(10, 10);
		ekfQ(12, 12) = ekfQ(10, 10);
	 


		//R
		mag3D_unitVector_meas = 0.01;
		pos_meas_m = 20;
		vel_meas_mps =0.2;
		ekfR.setZero();
		ekfR (0,0)= mag3D_unitVector_meas*mag3D_unitVector_meas;
		ekfR(1, 1) = ekfR(0, 0);
		ekfR(2, 2) = ekfR(0, 0);
 
		ekfR(3, 3) = vel_meas_mps*vel_meas_mps;
		ekfR(4, 4) = ekfR(3, 3);
		ekfR(5, 5) = ekfR(3, 3);
		 


		//初始化P

		//初始化P
	/*	ekfP.setIdentity();
		ekfP(0, 0) = 1000000 * 0.004f*0.004f;
		ekfP(1, 1) = ekfP(0, 0);
		ekfP(2, 2) = ekfP(0, 0);
		ekfP(3, 3) = ekfP(0, 0);
		ekfP(4, 4) = 10000 * 0.005f*0.005f;
		ekfP(5, 5) = ekfP(5, 5);
		ekfP(6, 6) = ekfP(5, 5);
		ekfP(7, 7) = 10000 * 0.000001f*0.000001f;
		ekfP(8, 8) = ekfP(7, 7);
		ekfP(9, 9) = ekfP(7, 7);
		ekfP(10, 10) = 10000 * 0.000000001f*0.000000001f;
		ekfP(11, 11) = ekfP(10, 10);
		ekfP(12, 12) = ekfP(10, 10);
		 */

		ekfP.setIdentity();
		ekfP(0, 0) = 16;
		ekfP(1, 1) = ekfP(0, 0);
		ekfP(2, 2) = ekfP(0, 0);
		ekfP(3, 3) = ekfP(0, 0);
		ekfP(4, 4) = 100;
		ekfP(5, 5) = ekfP(4, 4);
		ekfP(6, 6) = ekfP(4, 4);
		ekfP(7, 7) =0.0000000000001;
		ekfP(8, 8) = ekfP(7, 7);
		ekfP(9, 9) = ekfP(7, 7);
		ekfP(10, 10) = 0.000000000001;
		ekfP(11, 11) = ekfP(10, 10);
		ekfP(12, 12) = ekfP(10, 10);
		 
	}
	float X_GPS_DISTANCE; // GPS安装到IMU的x距离mm
	float Z_GPS_Height;  //gps 安装高度
	short INS_DELAY_TIME;
	INS_DATA_BUFF_SAVE ins_old_data[100];
	matrix::Matrix<float, 13, 1>  xhat_k;// 
	matrix::Matrix<float, 6, 1> z_k;
	matrix::Matrix<float, 13, 6>  ekfK;// 
	matrix::SquareMatrix<float, 13>  ekfP;//
	matrix::SquareMatrix<float, 13>  ekfQ;// 
	matrix::SquareMatrix<float, 6>  ekfR;//
	matrix::Matrix<float, 3, 1>  u1;// 
	matrix::Matrix<float, 3, 1>  u2;// 
	matrix::Matrix<float, 3, 1>  line_acc;// 
	matrix::Matrix<float, 3, 1>  line_acc_n;// 
	matrix::Matrix<float, 3, 1>  line_acc_b;// 

	float  u3;// 重力向量
	float  u4;// imu安装误差（磁力计）
	matrix::SquareMatrix<float, 3> C_ned2b_save;//姿态余弦矩阵
	matrix::SquareMatrix<float, 3> C_ned2b_body;//体系，没有考虑航向角
	matrix::SquareMatrix<float, 3> C_ned2b_body_new;//体系，没有考虑航向角
	float gravity_mps2;

	//R

	float mag3D_unitVector_meas;
	float pos_meas_m;
	float vel_meas_mps;


	float accel_meas_mps2;
	float gyro_bias_rps;

	//Q
	float Quaternion_process_noise;
	float pos_process_noise_m;
	float vel_process_noise_mps;
	float gyroBias_process_noise_rps;
	float accelBias_process_noise_mps2;



	float roll;
	float pith;
	float yaw;
	float roll_rad;
	float pith_rad;
	float yaw_rad;
	float test_vx;
	float test_vy;
	float yaw_MAG;
	 
	unsigned char Filter_ON;
	unsigned char GPS_is_valid;
	short lose_GPS_cnt;
	Vector3f ini_fixed_speed;  //消除GPS安转误差的原始速度  
	Vector3f gps_fix_err;  //消除GPS安转误差的原始速度
	matrix::Matrix<float, 3, 1> V_b_lag;
	 
	matrix::Matrix<float, 3, 1> V_forward;
	matrix::Matrix<float, 3, 1> V_b_forward;

	matrix::Matrix<float, 3, 1> mag3D_measure_in_body;
	void IMU_EKF2_init_ststes(float ROLL, float PITCH, float YAW, float gps_vx, float gps_vy, float gps_vz);
	void Imu_emistate_altitude_ekf(float T, float gx, float gy, float gz, float ax, float ay, float az,
		float mx, float my, float mz, float px, float py, float pz, float vx, float vy, float vz);

	void Imu_ekf(float T, float gx, float gy, float gz, float ax, float ay, float az,
		float yaw_MAG_rad, float px, float py, float pz, float vx, float vy, float vz);
	void Imu_ekf_q4_to_euler();
	void gps_speed_de_fix_err(float gx, float gy, float gz, float vx, float vy, float vz);
private:

};
extern IMU_EKF2 imu_ekf2;



#endif