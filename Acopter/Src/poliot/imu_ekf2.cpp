#include "imu_ekf2.h"
#include "filter.h"
#include "imu.h"
 
  IMU_EKF2 imu_ekf2;

#define pi 3.14159
#define GYRO_DEG_TORAD 0.017453292f

  void IMU_EKF2::Imu_emistate_altitude_ekf(float T, float gx, float gy, float gz, float ax, float ay, float az,
	  float mx, float my, float mz, float px, float py, float pz, float vx, float vy, float vz)
  {
	  float yaw_MAG_rad_ini;
	  //陀螺仪单位转化
	  gx = gx*GYRO_DEG_TORAD;
	  gy = gy*GYRO_DEG_TORAD;
	  gz = gz*GYRO_DEG_TORAD;
	  //消除GPS安装误差
	  gps_speed_de_fix_err(ins_old_data[INS_DELAY_TIME].g.x, ins_old_data[INS_DELAY_TIME].g.y, ins_old_data[INS_DELAY_TIME].g.z, vx, vy, vz);
	  //ACC
	  float acc_norm = 16384;
	  ax = -ax*gravity_mps2 / acc_norm;
	  ay = -ay*gravity_mps2 / acc_norm;
	  az = -az*gravity_mps2 / acc_norm;//反向
	 
 
	   yaw_MAG_rad_ini = -imu_dcm.Yaw / 57.3;
	//  yaw_MAG_rad_ini = -imu_dcm.yaw_mag/ 57.3;
	  yaw_MAG = imu_dcm.yaw_mag;

	  px = -px;
	  vx = -vx  ;
	  vy = vy  ;
	  vz = -vz  ;

	  vx = -ini_fixed_speed.x; 
	  vy = ini_fixed_speed.y;
	  vz = -ini_fixed_speed.z;

	  //滤波器参数
	  if (Filter_ON < 2)
	  {
		  mag3D_unitVector_meas = 1;
		  vel_meas_mps = 1;
	  }
	  else
	  {
		  mag3D_unitVector_meas = 0.01;
		  vel_meas_mps = 0.06;
	  }


	 
	  ekfQ(0, 0) = Quaternion_process_noise*Quaternion_process_noise;
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

	  ekfR(0, 0) = mag3D_unitVector_meas*mag3D_unitVector_meas;
	  ////倾角过大时不相信磁力计
	  //if (ABS(roll)>8||ABS(pith)>10)
	  //{
		 // ekfR(0, 0) = ekfR(0, 0) * 100;
	  //}
	  ekfR(1, 1) = ekfR(0, 0);
	  ekfR(2, 2) = ekfR(0, 0);
	  ekfR(3, 3) = vel_meas_mps*vel_meas_mps;
	  ekfR(4, 4) = ekfR(3, 3);
	  ekfR(5, 5) = ekfR(3, 3);

	  //存储旧值
	  for (int i = 99; i > 0;i--)
	  {
		  ins_old_data[i] = ins_old_data[i - 1];
	  }
	  ins_old_data[0].delta_T = T;
	  ins_old_data[0].g.x = gx;
	  ins_old_data[0].g.y = gy;
	  ins_old_data[0].g.z = gz;
	  ins_old_data[0].a.x = ax;
	  ins_old_data[0].a.y = ay;
	  ins_old_data[0].a.z = az;
	  ins_old_data[0].Yaw_mag_old = yaw_MAG_rad_ini;
	  Imu_ekf(T, ins_old_data[INS_DELAY_TIME].g.x, ins_old_data[INS_DELAY_TIME].g.y, ins_old_data[INS_DELAY_TIME].g.z,
		  ins_old_data[INS_DELAY_TIME].a.x, ins_old_data[INS_DELAY_TIME].a.y, ins_old_data[INS_DELAY_TIME].a.z,
		  ins_old_data[INS_DELAY_TIME].Yaw_mag_old,
		  px, py, pz,
		  vx, vy, vz

		  );
	  Imu_ekf_q4_to_euler();

  }



  void IMU_EKF2::Imu_ekf(float T, float gx, float gy, float gz,
	  float ax, float ay, float az,
	  float  yaw_MAG_rad,
	  float px, float py, float pz,
	  float vx, float vy, float vz)
  {
	  u3 = 9.81f;//重力
	  float	g_mps2 = 9.81f;//重力
	  u4 = 0.0f;//IMU安装误差？
	  float	mag_declination_deg = u4;

	  u1(0, 0) = gx;
	  u1(1, 0) = gy;
	  u1(2, 0) = gz;


	  /************************************************************************/
	  /* 将导航坐标的磁力分量投影                                                                     */
	  /************************************************************************/
	  //float yaw_MAG = fast_atan2(-my, mx);
	  float C_ned2b_without_yaw_data[9] = { cos(pith_rad)*cos(yaw_MAG_rad), cos(pith_rad)*sin(yaw_MAG_rad), -sin(pith_rad),
		  sin(roll_rad)*sin(pith_rad)*cos(yaw_MAG_rad) - cos(roll_rad)*sin(yaw_MAG_rad), sin(roll_rad)*sin(pith_rad)*sin(yaw_MAG_rad) + cos(roll_rad)*cos(yaw_MAG_rad), sin(roll_rad)*cos(pith_rad),
		  cos(roll_rad)*sin(pith_rad)*cos(yaw_MAG_rad) + sin(roll_rad)*sin(yaw_MAG_rad), cos(roll_rad)*sin(pith_rad)*sin(yaw_MAG_rad) - sin(roll_rad)*cos(yaw_MAG_rad), cos(roll_rad)*cos(pith_rad) };
	  matrix::SquareMatrix<float, 3 > C_ned2b_without_yaw(C_ned2b_without_yaw_data);
	  mag3D_measure_in_body.setZero();
	  mag3D_measure_in_body(0, 0) = 1;//[ 1 0 0]
	  mag3D_measure_in_body = C_ned2b_without_yaw* mag3D_measure_in_body;
	

	  //mag3D_measure_in_body = C_ned2b_without_yaw* mag3D_measure_in_body;
	  /************************************************************************/
	  /* 将导航坐标的磁力分量投影结束*/
	  /************************************************************************/


	  float z_k_data[6] = { mag3D_measure_in_body(0, 0), mag3D_measure_in_body(1, 0), mag3D_measure_in_body(2, 0),
		   vx, vy, vz };// 加表与磁力计测量
	  matrix::Matrix<float, 6, 1>  z_k(z_k_data);// 加表与磁力计测量
	
	  float  q0 = xhat_k(0, 0);
	  float  q1 = xhat_k(1, 0);
	  float  q2 = xhat_k(2, 0);
	  float  q3 = xhat_k(3, 0);

	/*  float   Pn = xhat_k(4, 0);
	  float   Pe = xhat_k(5, 0);
	  float   Alt = xhat_k(6, 0);*/
	  float   Vn = xhat_k(4, 0);
	  float   Ve = xhat_k(5, 0);
	  float   Vd = xhat_k(6, 0);
	  float   bwx = xhat_k(7, 0);
	  float   bwy = xhat_k(8, 0);
	  float   bwz = xhat_k(9, 0);

	  float   bax = xhat_k(10, 0);
	  float   bay = xhat_k(11, 0);
	  float   baz = xhat_k(12, 0);
	  float wx = gx;
	  float wy = gy;
	  float wz = gz;

	  float fx = ax;
	  float fy = ay;
	  float fz = az;

	  /************************************************************************/
	  /* 一步预测                                                                     */
	  /************************************************************************/
	  float C_bodyrate2qdot_data[12] = { -q1, -q2, -q3,
		  q0, -q3, q2,
		  q3, q0, -q1,
		  -q2, q1, q0 };
	  matrix::Matrix<float, 4, 3> C_bodyrate2qdot(C_bodyrate2qdot_data);
	  C_bodyrate2qdot = C_bodyrate2qdot*0.5f;
	  float C_ned2b_data[9] = { 1 - 2 * (q2 *q2 + q3*q3), 2 * (q1*q2 + q3*q0), 2 * (q1*q3 - q2*q0),
		  2 * (q1*q2 - q3*q0), 1 - 2 * (q1 *q1 + q3*q3), 2 * (q2*q3 + q1*q0),
		  2 * (q1*q3 + q2*q0), 2 * (q2*q3 - q1*q0), 1 - 2 * (q1*q1 + q2*q2) };
	  matrix::SquareMatrix<float, 3> C_ned2b(C_ned2b_data);
	  matrix::SquareMatrix<float, 3> C_b2ned = C_ned2b.transpose();
	 // C_ned2b_save = C_ned2b;
	  //状态的微分

	  //四元数的微分
	  matrix::Matrix<float, 3, 1> gy_de_bia;
	  gy_de_bia(0, 0) = wx - bwx;
	  gy_de_bia(1, 0) = wy - bwy;
	  gy_de_bia(2, 0) = wz - bwz;
	  matrix::Matrix<float, 4, 1> q_dot = C_bodyrate2qdot*gy_de_bia;
	  //导航系的加速度
	  matrix::Matrix<float, 3, 1> acc_dot;
	  acc_dot(0, 0) = fx - bax;
	  acc_dot(1, 0) = fy - bay;
	  acc_dot(2, 0) = fz - baz;
	  acc_dot = C_b2ned * acc_dot;
	  acc_dot(2, 0) = g_mps2 + acc_dot(2, 0);
	  line_acc_n = acc_dot;
	  matrix::Matrix<float, 13, 1> xdot;
	  xdot.setZero();
	  xdot(0, 0) = q_dot(0, 0); 
	  xdot(1, 0) = q_dot(1, 0);
	  xdot(2, 0) = q_dot(2, 0);
	  xdot(3, 0) = q_dot(3, 0);
	/*  xdot(4, 0) = Vn; 
	  xdot(5, 0) = Ve; 
	  xdot(6, 0) = -Vd;*/
	  xdot(4, 0) = acc_dot(0, 0); 
	  xdot(5, 0) = acc_dot(1, 0); 
	  xdot(6, 0) = acc_dot(2, 0);
	  //状态的微分
	  xhat_k = xhat_k + xdot*T;//一步状态预测

	  /* 计算F正                                                                     */
	  /************************************************************************/
	  float F_data[169] = {

		  0, bwx / 2 - wx / 2, bwy / 2 - wy / 2, bwz / 2 - wz / 2, 0, 0, 0, q1 / 2, q2 / 2, q3 / 2, 0, 0, 0,
		  wx / 2 - bwx / 2, 0, wz / 2 - bwz / 2, bwy / 2 - wy / 2,  0, 0, 0, -q0 / 2, q3 / 2, -q2 / 2, 0, 0, 0,
		  wy / 2 - bwy / 2, bwz / 2 - wz / 2, 0, wx / 2 - bwx / 2,  0, 0, 0, -q3 / 2, -q0 / 2, q1 / 2, 0, 0, 0,
		  wz / 2 - bwz / 2, wy / 2 - bwy / 2, bwx / 2 - wx / 2, 0, 0, 0, 0, q2 / 2, -q1 / 2, -q0 / 2, 0, 0, 0,
		  2 * q3*(bay - fy) - 2 * q2*(baz - fz), -2 * q2*(bay - fy) - 2 * q3*(baz - fz), 4 * q2*(bax - fx) - 2 * q1*(bay - fy) - 2 * q0*(baz - fz), 2 * q0*(bay - fy) + 4 * q3*(bax - fx) - 2 * q1*(baz - fz),  0, 0, 0, 0, 0, 0, 2 * q2 *q2 + 2 * q3 *q3 - 1, 2 * q0*q3 - 2 * q1*q2, -2 * q0*q2 - 2 * q1*q3,
		  2 * q1*(baz - fz) - 2 * q3*(bax - fx), 4 * q1*(bay - fy) - 2 * q2*(bax - fx) + 2 * q0*(baz - fz), -2 * q1*(bax - fx) - 2 * q3*(baz - fz), 4 * q3*(bay - fy) - 2 * q0*(bax - fx) - 2 * q2*(baz - fz),  0, 0, 0, 0, 0, 0, -2 * q0*q3 - 2 * q1*q2, 2 * q1*q1 + 2 * q3 *q3 - 1, 2 * q0*q1 - 2 * q2*q3,
		  2 * q2*(bax - fx) - 2 * q1*(bay - fy), 4 * q1*(baz - fz) - 2 * q3*(bax - fx) - 2 * q0*(bay - fy), 2 * q0*(bax - fx) - 2 * q3*(bay - fy) + 4 * q2*(baz - fz), -2 * q1*(bax - fx) - 2 * q2*(bay - fy),  0, 0, 0, 0, 0, 0, 2 * q0*q2 - 2 * q1*q3, -2 * q0*q1 - 2 * q2*q3, 2 * q1 *q1 + 2 * q2 *q2 - 1,
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  
		  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  

	  };
	  matrix::SquareMatrix<float, 13>  F(F_data);// 
	  matrix::SquareMatrix<float, 13>  I;// 
	  I.setIdentity();
	  F = F*T + I;
	  //预测ekfP
	  ekfP = F*ekfP*F.transpose() + ekfQ;
	  /************************************************************************/
	  /* 一步预测结束                                                                     */
	  /************************************************************************/

	  /************************************************************************/
	  /* 观测                                                                     */
	  /************************************************************************/
	  /*float C_ned2b_data[9] = { 1 - 2 * (q2 *q2 + q3*q3), 2 * (q1*q2 + q3*q0), 2 * (q1*q3 - q2*q0),
	  2 * (q1*q2 - q3*q0), 1 - 2 * (q1 *q1 + q3*q3), 2 * (q2*q3 + q1*q0),
	  2 * (q1*q3 + q2*q0), 2 * (q2*q3 - q1*q0), 1 - 2 * (q1*q1 + q2*q2) };
	  matrix::SquareMatrix<float, 3> C_ned2b(C_ned2b_data);*/
	if (GPS_is_valid)
	//  if (1)
	{
		GPS_is_valid = 0;
		lose_GPS_cnt = 0;

		matrix::SquareMatrix<float, 3> C_mag2ned;
		C_mag2ned.setIdentity();
		matrix::Matrix<float, 3, 1> mag3D_unitVector_in_body;
		mag3D_unitVector_in_body.setZero();
		mag3D_unitVector_in_body(0, 0) = 1.0f;//[1 ;0;0]
		mag3D_unitVector_in_body = C_ned2b*C_mag2ned*mag3D_unitVector_in_body;

		matrix::Matrix<float, 6, 1> zhat;

		zhat(0, 0) = mag3D_unitVector_in_body(0, 0);
		zhat(1, 0) = mag3D_unitVector_in_body(1, 0);
		zhat(2, 0) = mag3D_unitVector_in_body(2, 0);
		/*zhat(3, 0) = Pn;
		zhat(4, 0) = Pe;
		zhat(5, 0) = Alt;*/
		zhat(3, 0) = Vn;
		zhat(4, 0) = Ve;
		zhat(5, 0) = Vd;


		float H_data[78] = {

			2 * q3*sin((pi*mag_declination_deg) / 180), 2 * q2*sin((pi*mag_declination_deg) / 180), 2 * q1*sin((pi*mag_declination_deg) / 180) - 4 * q2*cos((pi*mag_declination_deg) / 180), 2 * q0*sin((pi*mag_declination_deg) / 180) - 4 * q3*cos((pi*mag_declination_deg) / 180), 0, 0, 0, 0, 0, 0, 0, 0, 0,
			(-2)*q3*cos((pi*mag_declination_deg) / 180), 2 * q2*cos((pi*mag_declination_deg) / 180) - 4 * q1*sin((pi*mag_declination_deg) / 180), 2 * q1*cos((pi*mag_declination_deg) / 180), -2 * q0*cos((pi*mag_declination_deg) / 180) - 4 * q3*sin((pi*mag_declination_deg) / 180), 0, 0, 0, 0, 0, 0, 0, 0, 0,
			// 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			2 * q2*cos((pi*mag_declination_deg) / 180) - 2 * q1*sin((pi*mag_declination_deg) / 180), 2 * q3*cos((pi*mag_declination_deg) / 180) - 2 * q0*sin((pi*mag_declination_deg) / 180), 2 * q0*cos((pi*mag_declination_deg) / 180) + 2 * q3*sin((pi*mag_declination_deg) / 180), 2 * q1*cos((pi*mag_declination_deg) / 180) + 2 * q2*sin((pi*mag_declination_deg) / 180), 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			/*	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,*/

		};
		matrix::Matrix<float, 6, 13>  H(H_data);// 
		matrix::SquareMatrix<float, 6>  Inv_save;// 
		Inv_save = (H*ekfP*H.transpose() + ekfR);

		ekfK = (ekfP*H.transpose())*Inv_save.I();
		xhat_k = xhat_k + ekfK*(z_k - zhat);
		I.setIdentity();
		ekfP = (I - ekfK*H)*ekfP;

		//归一化
		float norm_q = my_sqrt(xhat_k(0, 0)*  xhat_k(0, 0) + xhat_k(1, 0) * xhat_k(1, 0) +
			xhat_k(2, 0)* xhat_k(2, 0) + xhat_k(3, 0)*  xhat_k(3, 0));
		xhat_k(0, 0)=xhat_k(0, 0) / norm_q;
		xhat_k(1, 0)=xhat_k(1, 0) / norm_q;
		xhat_k(2, 0)=xhat_k(2, 0) / norm_q;
		xhat_k(3, 0)=xhat_k(3, 0) / norm_q;
	}
	else
	{
		lose_GPS_cnt++;
			if (lose_GPS_cnt>20)
			{
				lose_GPS_cnt = 0;
				GPS_is_valid = 1;
			}

	}
	 
	  
  }

  void IMU_EKF2::Imu_ekf_q4_to_euler()
  {
	 
	  /************************************************************************/
	  /* 速度体系                                                                     */
	  /************************************************************************/
	  V_forward(0,0) = imu_ekf2.xhat_k(4, 0);
	  V_forward(1, 0) = imu_ekf2.xhat_k(5, 0);
	  V_forward(2, 0) = imu_ekf2.xhat_k(6, 0);

	
	  float  q0 = xhat_k(0, 0);
	  float  q1 = xhat_k(1, 0);
	  float  q2 = xhat_k(2, 0);
	  float  q3 = xhat_k(3, 0);
	 
	  float q_data[4] = { q0, q1, q2, q3 };
	  matrix::Matrix<float, 4, 1> q_vect_4(q_data);
	  matrix::Matrix<float, 3, 1> acc_dot;
	 // 向前外推
	  for (int i = (INS_DELAY_TIME - 1); i >= 0;i--)
	  {
		  /************************************************************************/
		  float C_bodyrate2qdot_data[12] = { -q1, -q2, -q3,
			  q0, -q3, q2,
			  q3, q0, -q1,
			  -q2, q1, q0 };
		  matrix::Matrix<float, 4, 3> C_bodyrate2qdot(C_bodyrate2qdot_data);
		  C_bodyrate2qdot = C_bodyrate2qdot*0.5f;
		  
		  //四元数的微分
		  matrix::Matrix<float, 3, 1> gy_de_bia;
		  gy_de_bia(0, 0) = ins_old_data[i].g.x - xhat_k(7,0);
		  gy_de_bia(1, 0) = ins_old_data[i].g.y - xhat_k(8, 0);
		  gy_de_bia(2, 0) = ins_old_data[i].g.z - xhat_k(9, 0);
		  matrix::Matrix<float, 4, 1> q_dot = C_bodyrate2qdot*gy_de_bia;
		  q_vect_4 = q_vect_4 + q_dot* ins_old_data[i].delta_T;
		  float norm_q = my_sqrt(q_vect_4(0, 0)*  q_vect_4(0, 0) + q_vect_4(1, 0) * q_vect_4(1, 0) +
			  q_vect_4(2, 0)* q_vect_4(2, 0) + q_vect_4(3, 0)*  q_vect_4(3, 0));
		  q_vect_4 = q_vect_4 / norm_q;
		  q0 = q_vect_4(0, 0);
		  q1 = q_vect_4(1, 0);
		  q2 = q_vect_4(2, 0);
		  q3 = q_vect_4(3, 0);
		  float C_ned2b_data[9] = { 1 - 2 * (q2 *q2 + q3*q3), 2 * (q1*q2 + q3*q0), 2 * (q1*q3 - q2*q0),
			  2 * (q1*q2 - q3*q0), 1 - 2 * (q1 *q1 + q3*q3), 2 * (q2*q3 + q1*q0),
			  2 * (q1*q3 + q2*q0), 2 * (q2*q3 - q1*q0), 1 - 2 * (q1*q1 + q2*q2) };

		  matrix::SquareMatrix<float, 3> C_ned2b(C_ned2b_data);
		  C_ned2b_save = C_ned2b;	
		  acc_dot(0, 0) = ins_old_data[i].a.x - xhat_k(10, 0);
		  acc_dot(1, 0) = ins_old_data[i].a.y - xhat_k(11, 0);
		  acc_dot(2, 0) = ins_old_data[i].a.z - xhat_k(12, 0);
		  acc_dot = C_ned2b.transpose() * acc_dot;
		  acc_dot(2, 0) = 9.81 + acc_dot(2, 0);
		  V_forward = V_forward + acc_dot* ins_old_data[i].delta_T;

	  }
	  //保留最新的线运动（导航系）
	  line_acc = acc_dot;
	  roll_rad = fast_atan2(C_ned2b_save(1, 2), C_ned2b_save(2, 2));
	  pith_rad = asin(-C_ned2b_save(0, 2));
	  yaw_rad = fast_atan2(C_ned2b_save(0, 1), C_ned2b_save(0, 0));

	  roll = 180 / pi * roll_rad;
	  pith = -180 / pi *pith_rad;
	  yaw =-180 / pi * yaw_rad; // -180 <= yaw <= 180
	  if (ABS(C_ned2b_save(0, 2))>0.9999999)
	  {
		  roll_rad = 0;
		  pith_rad = fast_atan2(-C_ned2b_save(0, 2), C_ned2b_save(2, 2));
		  yaw_rad = fast_atan2(-C_ned2b_save(1, 0), C_ned2b_save(1, 1));
		  roll = 0;
		  pith = -180 / pi*pith_rad;
		  yaw = -180 / pi*yaw_rad;
	  }

	  //不考虑航向的情
	   
	  //速度向机体系分解
	  V_b_lag(0, 0) = imu_ekf2.xhat_k(4, 0)*cos(imu_ekf2.yaw*pi / 180.0f) - imu_ekf2.xhat_k(5, 0) *sin(imu_ekf2.yaw*pi / 180.0f);
	  V_b_lag(1, 0) = imu_ekf2.xhat_k(4, 0)*sin(imu_ekf2.yaw*pi / 180.0f) + imu_ekf2.xhat_k(5, 0)*cos(imu_ekf2.yaw*pi / 180.0f);
	  V_b_lag(2, 0) = imu_ekf2.xhat_k(6, 0);

	  V_b_forward(0, 0) = V_forward(0, 0)*cos(imu_ekf2.yaw*pi / 180.0f) - V_forward(1, 0) *sin(imu_ekf2.yaw*pi / 180.0f);
	  V_b_forward(1, 0) = V_forward(0, 0)*sin(imu_ekf2.yaw*pi / 180.0f) + V_forward(1, 0)*cos(imu_ekf2.yaw*pi / 180.0f);
	  V_b_forward(2, 0) = V_forward(2, 0);
	  //加速度向机体系分解
	  line_acc_b(0, 0) = line_acc(0, 0)*cos(imu_ekf2.yaw*pi / 180.0f) - line_acc(1, 0) *sin(imu_ekf2.yaw*pi / 180.0f);
	  line_acc_b(1, 0) = line_acc(0, 0)*sin(imu_ekf2.yaw*pi / 180.0f) + line_acc(1, 0)*cos(imu_ekf2.yaw*pi / 180.0f);
	  line_acc_b(2, 0) = line_acc(2, 0);
  }



  /************************************************************************/
  /* gps安装误差引起的杆臂效应                                                                     */
  /************************************************************************/
#define GYRO_DEG_TORAD 0.017453292f
  void IMU_EKF2::gps_speed_de_fix_err(float gx, float gy, float gz, float vx, float vy, float vz)
  {
	  //陀螺仪单位转化
	  //gx = gx*GYRO_DEG_TORAD;
	  //gy = gy*GYRO_DEG_TORAD;
	  //gz = gz*GYRO_DEG_TORAD;
	 // Vector3f err_v;
	 

	float  vx_b = vx*cos(imu_ekf2.yaw*pi / 180.0f) + vy *sin(imu_ekf2.yaw*pi / 180.0f);
	float  vy_b = -vx *sin(imu_ekf2.yaw*pi / 180.0f) + vy *cos(imu_ekf2.yaw*pi / 180.0f);



	  gps_fix_err.x = gy*Z_GPS_Height;
	  gps_fix_err.y = (-gz*X_GPS_DISTANCE) + gx * Z_GPS_Height;
	  gps_fix_err.z = (-gy*X_GPS_DISTANCE);

	  vx_b = vx_b - gps_fix_err.x;
	  vy_b = vy_b - gps_fix_err.y;


	  ini_fixed_speed.x = vx_b*cos(imu_ekf2.yaw*pi / 180.0f)- vy_b *sin(imu_ekf2.yaw*pi / 180.0f);
	  ini_fixed_speed.y = vx_b*sin(imu_ekf2.yaw*pi / 180.0f) + vy_b *cos(imu_ekf2.yaw*pi / 180.0f);;
	  ini_fixed_speed.z = vz - gps_fix_err.z;

  }

  void IMU_EKF2::IMU_EKF2_init_ststes(float ROLL, float PITCH, float YAW, float gps_vx, float gps_vy, float gps_vz )
  {
	  float phi = ROLL *pi/180.f;
	  float theta =- PITCH *pi / 180.f;
	  float psi= -YAW  *pi / 180.f;
	  imu_ekf2.xhat_k(0, 0) = cos(psi / 2)*cos(theta / 2)*cos(phi / 2) + sin(psi / 2)*sin(theta / 2)*sin(phi / 2);
	  imu_ekf2.xhat_k(1, 0) = cos(psi / 2)*cos(theta / 2)*sin(phi / 2) - sin(psi / 2)*sin(theta / 2)*cos(phi / 2);
	  imu_ekf2.xhat_k(2, 0) = cos(psi / 2)*sin(theta / 2)*cos(phi / 2) + sin(psi / 2)*cos(theta / 2)*sin(phi / 2);
	  imu_ekf2.xhat_k(3, 0) = sin(psi / 2)*cos(theta / 2)*cos(phi / 2) - cos(psi / 2)*sin(theta / 2)*sin(phi / 2);
	  imu_ekf2.xhat_k(4, 0) =- gps_vx; 
	  imu_ekf2.xhat_k(5, 0) = gps_vy;
	  imu_ekf2.xhat_k(6, 0) = -gps_vz;
  }