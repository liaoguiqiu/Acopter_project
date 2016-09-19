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

	float Roll, Pitch, Yaw;    				//вкл╛╫г

	float ref_q[4]  ;
	float norm_acc, norm_q;
	float norm_acc_lpf;

	float mag_norm, mag_norm_xyz;

	Vector3f mag_sim_3d;

	float yaw_mag;

	IMU_DCM()
	{
		Kp = 0.6;
		Ki = 0.05;
		ref_q[0] = 1;
		ref_q[1] = 0;
		ref_q[2] = 0;
		ref_q[3] = 0;
	}

	void IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay, float az, float *rol, float *pit, float *yaw);
private:


};


extern IMU_DCM imu_dcm;



#endif

