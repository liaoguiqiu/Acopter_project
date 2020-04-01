#include "gps_kalman_v4.h"
#include "mymath.h"
GPS_KAMAN_V4 gps_po_kf_v4;

void GPS_KAMAN_V4::gps_pos_v4_fuse(float T, float px_b, float py_b, float vx_b1, float vy_b1)
{
	x_pos_kf.gps_makof_KF(T, px_b, my_deathzoom(vx_b1,100));
	y_pos_kf.gps_makof_KF(T, py_b,my_deathzoom( vy_b1,100));

	x_b = x_pos_kf.X(0, 0);
	y_b = y_pos_kf.X(0, 0);
	vx_b = x_pos_kf.X(1, 0);
	vy_b = y_pos_kf.X(1, 0);

}
/************************************************************************/
/* 使用积分的方式来消除gps速度与融合速度之间静差                                                                     */
/************************************************************************/
void GPS_KAMAN_V4::speed_and_acc_debias(float T, float vx_fward, float vy_fward, float corect_vx, float corect_vy,float acc_x,float acc_y)
{
	/*if (ABS(corect_vx) < 100)
	{
		I.x = (corect_vx - vx_fward);
	}
	if (ABS(corect_vy) < 100)
	{
		I.y = (corect_vy - vy_fward);
	}*/
	I.x += ki*( (corect_vx - vx_fward)- I.x);
	I.y += ki*((corect_vy - vy_fward) - I.y);

	//低通滤波
	 
	speed_b_debias.x = vx_fward + I.x;
	speed_b_debias.y = vy_fward + I.y;
	/*speed_b_debias.x += (1 / (1 + 1 / (lf_filter2 *3.14f *T))) *(vx_fward + I.x - speed_b_debias.x);
	speed_b_debias.y += (1 / (1 + 1 / (lf_filter2 *3.14f *T))) *(vy_fward + I.y - speed_b_debias.y);
*/

	float ini_acc_x, ini_acc_y;
	ini_acc_x = (speed_b_debias.x - speed_b_debias_old[1].x )/T/2;
	ini_acc_y = (speed_b_debias.y - speed_b_debias_old[1].y) / T/2;

	speed_b_debias_old[1].x = speed_b_debias_old[0].x;
	speed_b_debias_old[1].y = speed_b_debias_old[0].y;

 	speed_b_debias_old[0].x = speed_b_debias.x; 
	speed_b_debias_old[0].y = speed_b_debias.y;

	/*acc_b_debias.x += (1 / (1 + 1 / (lf_filter *3.14f *T))) *(ini_acc_x - acc_b_debias.x);
	acc_b_debias.y += (1 / (1 + 1 / (lf_filter *3.14f *T))) *(ini_acc_y - acc_b_debias.y);*/
	acc_b_debias.x += (1 / (1 + 1 / (lf_filter *3.14f *T))) *(acc_x - acc_b_debias.x);
	acc_b_debias.y += (1 / (1 + 1 / (lf_filter *3.14f *T))) *(acc_y - acc_b_debias.y);

}
