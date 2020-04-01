#ifndef _IMU_H_
#define	_IMU_H_

#include "include.h"
#include "parameter.h"
#include "mymath.h"
#include "math.h"
#include "vector3.h"


#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1	

typedef struct
{
	Vector3f err;
	Vector3f err_tmp;
	Vector3f err_lpf;
	Vector3f err_Int;
	Vector3f g;

}ref_t;


class IMU_DCM
{
public:
	float  Kp  ;
	float  Ki  ;
	ref_t ref;
	Vector3f  reference_v;
	float a_x, a_y, a_z, b_x, b_y, b_z, c_x, c_y, c_z;       //由三个姿态角解算出来的DCM矩阵
	float Roll, Pitch, Yaw;    				//姿态角
	float roll_rad, pit_rad, yaw_rad;
	float ref_q[4]  ;
	float norm_acc, norm_q;
	float norm_acc_lpf;

	float mag_norm, mag_norm_xyz;

	Vector3f mag_sim_3d;

	float yaw_mag;

	IMU_DCM()
	{
		Kp = 0.8;
		Ki = 0.01;
		ref_q[0] = 1;
		ref_q[1] = 0;
		ref_q[2] = 0;
		ref_q[3] = 0;
	}

	void IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay, float az, float *rol, float *pit, float *yaw);
	void from_euler(float roll, float pitch, float yaw, float &a_x, float &a_y, float &a_z, float &b_x, float &b_y, float &b_z, float &c_x, float &c_y, float &c_z);

private:


};


extern IMU_DCM imu_dcm;



#endif

