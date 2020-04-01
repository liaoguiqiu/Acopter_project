#include "imu_kalman.h"
#include "filter.h"
#include "imu.h"
IMU_EXTEND_KALMAN  imu_ekf;

#define pi 3.14159
#define GYRO_DEG_TORAD 0.017453292f

void IMU_EXTEND_KALMAN::Imu_emistate_altitude_ekf(float T, float gx, float gy, float gz, float ax, float ay, float az,
	float mx, float my, float mz, float vx, float vy, float vz)
{
	//陀螺仪单位转化
	gx = gx*GYRO_DEG_TORAD;
	gy = gy*GYRO_DEG_TORAD;
	gz = gz*GYRO_DEG_TORAD;
	//ACC
	float acc_norm = my_sqrt(ax*ax + ay*ay + az*az);
	ax = -ax*gravity_mps2 / acc_norm;
	ay = -ay*gravity_mps2 / acc_norm;
	az = -az*gravity_mps2 / acc_norm;//反向
	//mag
	/*mx = 0;
	my = 1;*/
	 
	float mag_norm = my_sqrt(mx*mx + my*my + mz*mz);

	Vector3f mag_temp(mx / mag_norm, my / mag_norm, mz / mag_norm );
	Vector3f reference_v(C_ned2b_save(0, 2), C_ned2b_save(1, 2), C_ned2b_save(2, 2));
	Vector3f mag_sim_3d;
	filter.simple_3d_trans(&reference_v, &mag_temp, &mag_sim_3d);

	mx = mag_sim_3d.x;
	my = mag_sim_3d.y;

	mag_norm = my_sqrt(mx*mx + my*my );
	if (mag_norm != 0)
	{
		mx = mx / mag_norm;
		my = my / mag_norm;
		yaw_MAG_rad = -fast_atan2(my, mx);
		yaw_MAG = -yaw_MAG_rad *57.3f;

	}
	mz = 0;
	vx = vx / 1000.0f;
	vy = vy / 1000.0f;
	vz = vz / 1000.0f;
	vx = 0;
	vy = 0;
	vz = 0;
	
	if (imu_dcm.norm_acc>8000 && imu_dcm.norm_acc <8500)
	{
		Q.setIdentity();
		Q = Q*Quaternion_process_noise*Quaternion_process_noise ;
		Q(4, 4) = gyroBias_process_noise_rps*gyroBias_process_noise_rps ;
		Q(5, 5) = gyroBias_process_noise_rps*gyroBias_process_noise_rps ;
		Q(6, 6) = gyroBias_process_noise_rps*gyroBias_process_noise_rps ;
		R.setIdentity();
		R = R*accel_meas_mps2*accel_meas_mps2;
		R(3, 3) = mag3D_unitVector_meas*mag3D_unitVector_meas;
		R(4, 4) = mag3D_unitVector_meas*mag3D_unitVector_meas;
		R(5, 5) = mag3D_unitVector_meas*mag3D_unitVector_meas;

	}
	else
	{
		Q.setIdentity();
		Q = Q*Quaternion_process_noise*Quaternion_process_noise*0.00000001;
		Q(4, 4) = gyroBias_process_noise_rps*gyroBias_process_noise_rps*0.0000001;
		Q(5, 5) = gyroBias_process_noise_rps*gyroBias_process_noise_rps*0.0000001;
		Q(6, 6) = gyroBias_process_noise_rps*gyroBias_process_noise_rps*0.0000001;
		R.setIdentity();
		R = R*accel_meas_mps2*accel_meas_mps2*10000;
		R(3, 3) = mag3D_unitVector_meas*mag3D_unitVector_meas;
		R(4, 4) = mag3D_unitVector_meas*mag3D_unitVector_meas;
		R(5, 5) = mag3D_unitVector_meas*mag3D_unitVector_meas;
	}


	Imu_ekf(T, gx, gy, gz,
		ax, ay, az,
		mx, my, mz,
		vx, vy, vz);
	Imu_ekf_q4_to_euler();

}



void IMU_EXTEND_KALMAN::Imu_ekf(float T, float gx, float gy, float gz,
	float ax, float ay, float az,
	float mx, float my, float mz,
	float vx, float vy, float vz)
{
	u3 = 9.81f;//重力

	u4 = 0.0f;//IMU安装误差？

	u1(0, 0) = gx;
	u1(1, 0) = gy;
	u1(2, 0) = gz;

	//速度
	u2(0, 0) = vx;
	u2(1, 0) = vy;
	u2(2, 0) = vz;


	/************************************************************************/
	/* 将导航坐标的磁力分量投影                                                                     */
	/************************************************************************/
	//float yaw_MAG = fast_atan2(-my, mx);
	float C_ned2b_without_yaw_data[9] = { cos(pith_rad)*cos(yaw_MAG_rad), cos(pith_rad)*sin(yaw_MAG_rad), -sin(pith_rad),
		sin(roll_rad)*sin(pith_rad)*cos(yaw_MAG_rad) - cos(roll_rad)*sin(yaw_MAG_rad), sin(roll_rad)*sin(pith_rad)*sin(yaw_MAG_rad) + cos(roll_rad)*cos(yaw_MAG_rad), sin(roll_rad)*cos(pith_rad),
		cos(roll_rad)*sin(pith_rad)*cos(yaw_MAG_rad) + sin(roll_rad)*sin(yaw_MAG_rad), cos(roll_rad)*sin(pith_rad)*sin(yaw_MAG_rad) - sin(roll_rad)*cos(yaw_MAG_rad), cos(roll_rad)*cos(pith_rad) };
	matrix::SquareMatrix<float, 3 > C_ned2b_without_yaw(C_ned2b_without_yaw_data);
	mag3D_measure_in_body.setZero();
	 mag3D_measure_in_body(1, 0) = 1;//[ 1 0 0]
	 mag3D_measure_in_body = C_ned2b_without_yaw* mag3D_measure_in_body;
 	//mag3D_measure_in_body = C_ned2b_without_yaw* mag3D_measure_in_body;
	/************************************************************************/
	/* 将导航坐标的磁力分量投影结束*/
	/************************************************************************/
	float z_k_data[6] = { ax, ay, az, mag3D_measure_in_body(0, 0), mag3D_measure_in_body(1, 0), mag3D_measure_in_body(2, 0) };// 加表与磁力计测量
	matrix::Matrix<float, 6, 1>  z_k(z_k_data);// 加表与磁力计测量
	float norm_q = my_sqrt(xhat_k(0, 0)*  xhat_k(0, 0) + xhat_k(1, 0) * xhat_k(1, 0) +
		xhat_k(2, 0)* xhat_k(2, 0) + xhat_k(3, 0)*  xhat_k(3, 0));
	xhat_k(0, 0) / norm_q;
	xhat_k(1, 0) / norm_q;
	xhat_k(2, 0) / norm_q;
	xhat_k(3, 0) / norm_q;
	float  q0 = xhat_k(0, 0);
	float  q1 = xhat_k(1, 0);
	float  q2 = xhat_k(2, 0);
	float  q3 = xhat_k(3, 0);

	float   bwx = xhat_k(4, 0);
	float   bwy = xhat_k(5, 0);
	float   bwz = xhat_k(6, 0);

	float wx = u1(0, 0);
	float wy = u1(1, 0);
	float wz = u1(2, 0);


	/************************************************************************/
	/* 一步预测                                                                     */
	/************************************************************************/
	float C_bodyrate2qdot_data[12] = { -q1, -q2, -q3,
		q0, -q3, q2,
		q3, q0, -q1,
		-q2, q1, q0 };
	matrix::Matrix<float, 4, 3> C_bodyrate2qdot(C_bodyrate2qdot_data);
	C_bodyrate2qdot = C_bodyrate2qdot*0.5f;

	u1(0, 0) = gx - bwx;
	u1(1, 0) = gy - bwy;
	u1(2, 0) = gz - bwz;
	//状态的微分
	matrix::Matrix<float, 4, 1> xdot0 = C_bodyrate2qdot*u1;
	matrix::Matrix<float, 7, 1> xdot;
	xdot.setZero();
	xdot(0, 0) = xdot0(0, 0);
	xdot(1, 0) = xdot0(1, 0);
	xdot(2, 0) = xdot0(2, 0);
	xdot(3, 0) = xdot0(3, 0);
	xhat_k = xhat_k + xdot*T;//一步状态预测

	/* 计算F正                                                                     */
	/************************************************************************/
	float F_data[49] = {
		0, bwx / 2 - wx / 2, bwy / 2 - wy / 2, bwz / 2 - wz / 2, q1 / 2, q2 / 2, q3 / 2,
		wx / 2 - bwx / 2, 0, wz / 2 - bwz / 2, bwy / 2 - wy / 2, -q0 / 2, q3 / 2, -q2 / 2,
		wy / 2 - bwy / 2, bwz / 2 - wz / 2, 0, wx / 2 - bwx / 2, -q3 / 2, -q0 / 2, q1 / 2,
		wz / 2 - bwz / 2, wy / 2 - bwy / 2, bwx / 2 - wx / 2, 0, q2 / 2, -q1 / 2, -q0 / 2,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0
	};
	matrix::SquareMatrix<float, 7>  F(F_data);// 
	matrix::SquareMatrix<float, 7>  I;// 
	I.setIdentity();
	F = F*T + I;
	//预测P
	P = F*P*F.transpose() + Q;
	/************************************************************************/
	/* 一步预测结束                                                                     */
	/************************************************************************/

	/************************************************************************/
	/* 观测                                                                     */
	/************************************************************************/
	float C_ned2b_data[9] = { 1 - 2 * (q2 *q2 + q3*q3), 2 * (q1*q2 + q3*q0), 2 * (q1*q3 - q2*q0),
		2 * (q1*q2 - q3*q0), 1 - 2 * (q1 *q1 + q3*q3), 2 * (q2*q3 + q1*q0),
		2 * (q1*q3 + q2*q0), 2 * (q2*q3 - q1*q0), 1 - 2 * (q1*q1 + q2*q2) };
	matrix::SquareMatrix<float, 3> C_ned2b(C_ned2b_data);
	C_ned2b_save = C_ned2b;
	matrix::SquareMatrix<float, 3> C_mag2ned;
	C_mag2ned.setIdentity();
	matrix::Matrix<float, 3, 1> mag3D_unitVector_in_body;
	mag3D_unitVector_in_body.setZero();
	mag3D_unitVector_in_body(0, 0) = 1.0f;//[1 ;0;0]
	mag3D_unitVector_in_body = C_ned2b*C_mag2ned*mag3D_unitVector_in_body;

	//引入速度修正 u1=g-bw
	/************************************************************************/
	/* accel_fb_mps2 =
	cross([wx-bwx; wy-bwy; wz-bwz], [Vx;Vy;Vz])
	- C_ned2b*[0;0;u3];
	*/
	/************************************************************************/
	matrix::Matrix<float, 3, 1> accel_fb_mps2;
	accel_fb_mps2(0, 0) = u1(1, 0)*vz - vy*u1(2, 0);
	accel_fb_mps2(1, 0) = u1(2, 0)*vx - vz*u1(0, 0);
	accel_fb_mps2(2, 0) = u1(0, 0)*vy - vx*u1(1, 0);
	matrix::Matrix<float, 3, 1> gravity;
	gravity.setZero();
	gravity(2, 0) = u3;
	accel_fb_mps2 = accel_fb_mps2 - C_ned2b*gravity;

	matrix::Matrix<float, 6, 1> zhat;
	zhat(0, 0) = accel_fb_mps2(0, 0);
	zhat(1, 0) = accel_fb_mps2(1, 0);
	zhat(2, 0) = accel_fb_mps2(2, 0);
	zhat(3, 0) = mag3D_unitVector_in_body(0, 0);
	zhat(4, 0) = mag3D_unitVector_in_body(1, 0);
	zhat(5, 0) = mag3D_unitVector_in_body(2, 0);

    float H_data[42] = {

	2 * u3*q2, (-2)*u3*q3, 2 * u3*q0, (-2)*u3*q1, 0, -vz, vy,
	(-2)*u3*q1, (-2)*u3*q0, (-2)*u3*q3, (-2)*u3*q2, vz, 0, -vx,
	0, 4 * u3*q1, 4 * u3*q2, 0, -vy, vx, 0,
	2 * q3* sin((pi*u4) / 180), 2 * q2*sin((pi*u4) / 180), 2 * q1*sin((pi*u4) / 180) - 4 * q2*cos((pi*u4) / 180), 2 * q0*sin((pi*u4) / 180) - 4 * q3*cos((pi*u4) / 180), 0, 0, 0,
	(-2)*q3*cos((pi*u4) / 180), 2 * q2*cos((pi*u4) / 180) - 4 * q1*sin((pi*u4) / 180), 2 * q1*cos((pi*u4) / 180), -2 * q0*cos((pi*u4) / 180) - 4 * q3*sin((pi*u4) / 180), 0, 0, 0,
	2 * q2*cos((pi*u4) / 180) - 2 * q1*sin((pi*u4) / 180), 2 * q3*cos((pi*u4) / 180) - 2 * q0*sin((pi*u4) / 180), 2 * q0*cos((pi*u4) / 180) + 2 * q3*sin((pi*u4) / 180), 2 * q1*cos((pi*u4) / 180) + 2 * q2*sin((pi*u4) / 180), 0, 0, 0,

	};

	//float H_data[42] = {

	//	2 * u3*q2, (-2)*u3*q3, 2 * u3*q0, (-2)*u3*q1, 0, -vz, vy,
	//	(-2)*u3*q1, (-2)*u3*q0, (-2)*u3*q3, (-2)*u3*q2, vz, 0, -vx,
	//	0, 4 * u3*q1, 4 * u3*q2, 0, -vy, vx, 0,
	//	2 * q3* sin((pi*u4) / 180), 2 * q2*sin((pi*u4) / 180), 2 * q1*sin((pi*u4) / 180) - 4 * q2*cos((pi*u4) / 180), 2 * q0*sin((pi*u4) / 180) - 4 * q3*cos((pi*u4) / 180), 0, 0, 0,
	//	(-2)*q3*cos((pi*u4) / 180), 2 * q2*cos((pi*u4) / 180) - 4 * q1*sin((pi*u4) / 180), 2 * q1*cos((pi*u4) / 180), -2 * q0*cos((pi*u4) / 180) - 4 * q3*sin((pi*u4) / 180), 0, 0, 0,
	//	0, 0, 0, 0, 0, 0, 0

	//};
	matrix::Matrix<float, 6, 7>  H(H_data);// 
	matrix::SquareMatrix<float, 6>  Inv_save;// 
	Inv_save = (H*P*H.transpose() + R);

	K = (P*H.transpose())*Inv_save.I();
	xhat_k = xhat_k + K*(z_k - zhat);
	I.setIdentity();
	P = (I - K*H)*P;
}

void IMU_EXTEND_KALMAN::Imu_ekf_q4_to_euler()
{

	roll_rad = fast_atan2(C_ned2b_save(1, 2), C_ned2b_save(2, 2));
	pith_rad = asin(-C_ned2b_save(0, 2));
	yaw_rad = fast_atan2(C_ned2b_save(0, 0), -C_ned2b_save(0, 1));

	roll = 180 / pi * roll_rad;
	pith = -180 / pi *pith_rad;
	yaw = -180 / pi * yaw_rad; // -180 <= yaw <= 180
	if (ABS(C_ned2b_save(0, 2))>0.9999999)
	{
		roll_rad = 0;
		pith_rad = fast_atan2(-C_ned2b_save(0, 2), C_ned2b_save(2, 2));
		yaw_rad = fast_atan2(C_ned2b_save(1, 1), C_ned2b_save(1, 0));
		roll = 0;
		pith = -180 / pi*pith_rad;
		yaw = -180 / pi*yaw_rad;
	}

}