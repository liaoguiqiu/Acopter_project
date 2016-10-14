#include "imu.h"
#include "ctrl.h"
#include "magnet.h"
#include "filter.h"
#include "rc.h"
#include "height_ctrl.h"
IMU_DCM imu_dcm;

void IMU_DCM:: IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay, float az, float *rol, float *pit, float *yaw)
{
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static Vector3f mag_tmp;


	mag_norm_tmp = 20 * (6.28f *half_T);

	mag_norm_xyz = my_sqrt(mag_s.Mag_Val.x * mag_s.Mag_Val.x + mag_s.Mag_Val.y * mag_s.Mag_Val.y + mag_s.Mag_Val.z * mag_s.Mag_Val.z);

	if (mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *((float)mag_s.Mag_Val.x / (mag_norm_xyz)-mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *((float)mag_s.Mag_Val.y / (mag_norm_xyz)-mag_tmp.y);
		mag_tmp.z += mag_norm_tmp *((float)mag_s.Mag_Val.z / (mag_norm_xyz)-mag_tmp.z);
	}

	filter.simple_3d_trans(&reference_v, &mag_tmp, &mag_sim_3d);

	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);

	if (mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag = fast_atan2((mag_sim_3d.y / mag_norm), (mag_sim_3d.x / mag_norm)) *57.3f;

		//非线性矫正 
		//   yaw_mag=  fit_mag_yaw(yaw_mag);
	}
	//=============================================================================
	// 计算等效重力向量
	reference_v.x = 2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]);
	reference_v.y = 2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]);
	reference_v.z = 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];


	//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。       
	//=============================================================================


	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);
	norm_acc_lpf += NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	if (ABS(ax)<8800 && ABS(ay)<8800 && ABS(az)<8800)
	{
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc_lpf;//4096.0f;
		ay = ay / norm_acc_lpf;//4096.0f;
		az = az / norm_acc_lpf;//4096.0f; 

		if (7600 < norm_acc && norm_acc < 8800)
		{
			/* 叉乘得到误差 */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
			ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;

			/* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *(ref.err_tmp.x - ref.err_lpf.x);
			ref.err_lpf.y += ref_err_lpf_hz *(ref.err_tmp.y - ref.err_lpf.y);
			ref.err_lpf.z += ref_err_lpf_hz *(ref.err_tmp.z - ref.err_lpf.z);

			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
			ref.err.z = ref.err_lpf.z;
		}
	}
	else
	{
		ref.err.x = 0;
		ref.err.y = 0;
		ref.err.z = 0;
	}
	/* 误差积分 */
	ref.err_Int.x += ref.err.x *Ki * 2 * half_T;
	ref.err_Int.y += ref.err.y *Ki * 2 * half_T;
	ref.err_Int.z += ref.err.z *Ki * 2 * half_T;

	/* 积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.y = LIMIT(ref.err_Int.y, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.z = LIMIT(ref.err_Int.z, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

	if (reference_v.z > 0.0f)
	{

		if (ctrl_s.thr>THR_BEFOR_FLY_UP )
		{
			yaw_correct = Kp *0.0f *To_180_degrees(yaw_mag - Yaw);
			//已经解锁，只需要低速纠正。
		}
		else
		{
			yaw_correct = Kp *0.0f *To_180_degrees(yaw_mag - Yaw);
			//没有解锁，视作开机时刻，快速纠正
		}
		// 		if( yaw_correct>360 || yaw_correct < -360  )
		// 		{
		// 			yaw_correct = 0;
		// 			//限制纠正范围+-360，配合+-180度取值函数
		// 		}

	}


	//	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	//	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	//	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN  + ( Kp*(ref.err.z + ref.err_Int.z) ) ;
	//	

	//  	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	//	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		  //IN RADIAN
	//	ref.g.z = gz;//(gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.z + ref.err_Int.z) );

	ref.g.x = (gx)*ANGLE_TO_RADIAN + (Kp*(ref.err.x + ref.err_Int.x));     //IN RADIAN
	ref.g.y = (gy)*ANGLE_TO_RADIAN + (Kp*(ref.err.y + ref.err_Int.y));		  //IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;

	/* 用叉积误差来做PI修正陀螺零偏 */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] + (-ref_q[1] * ref.g.x - ref_q[2] * ref.g.y - ref_q[3] * ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0] * ref.g.x + ref_q[2] * ref.g.z - ref_q[3] * ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0] * ref.g.y - ref_q[1] * ref.g.z + ref_q[3] * ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0] * ref.g.z + ref_q[1] * ref.g.y - ref_q[2] * ref.g.x)*half_T;

	/* 四元数规一化 normalise quaternion */
	norm_q = my_sqrt(ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2] + ref_q[3] * ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;




	*rol = fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) *57.3f;
	*pit = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2])) *57.3f;
	// 				//Yaw   = ( - fast_atan2(2*(ref_q[1]*ref_q[2] + ref_q[0]*ref_q[3]),ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] - ref_q[3]*ref_q[3]) )* 57.3;
	*yaw = fast_atan2(2 * (-ref_q[1] * ref_q[2] - ref_q[0] * ref_q[3]), 2 * (ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1]) - 1) *57.3f;// 
	// *yaw = yaw_mag;




}
