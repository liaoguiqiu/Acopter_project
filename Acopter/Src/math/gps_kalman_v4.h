#ifndef GPS_KALMAN_X_V4_H
#define GPS_KALMAN_X_V4_H
#include "kalman.h"
#include "vector3.h"
class GPS_KAMAN_V4
{

public:
	GPS_KAMAN_V4()
	{
		ki = 0.004;
		lf_filter = 1.0;
		lf_filter2  = 2.0;

	}
	GPS_POS_MAKOF_KF x_pos_kf;
	GPS_POS_MAKOF_KF y_pos_kf;
	float x_b;
	float y_b;
	float vx_b;
	float vy_b;
	float ki;
	float lf_filter;
	float lf_filter2;

	Vector3f I;
	Vector3f speed_b_debias;
	Vector3f speed_b_debias_old[3];
	Vector3f acc_b_debias;

	void speed_and_acc_debias(float T, float vx_fward, float vy_fward, float corect_vx, float corect_vy, float acc_x, float acc_y);
	void gps_pos_v4_fuse(float T, float px_b, float py_b, float vx_b1, float vy_b1);
	void gps_speed_de_fix_err(float gx,float gy,float gz,float vx, float vy ,float vz);
private:

};
extern GPS_KAMAN_V4 gps_po_kf_v4;

#endif