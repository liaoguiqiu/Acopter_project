#include "imu.h"
#include "ctrl.h"
#include "magnet.h"
#include "filter.h"
#include "rc.h"
#include "height_ctrl.h"

#include "imu_ekf2.h"

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

		//�����Խ��� 
		//   yaw_mag=  fit_mag_yaw(yaw_mag);
	}
	//=============================================================================
	// �����Ч��������
	reference_v.x = 2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]);
	reference_v.y = 2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]);
	reference_v.z = 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];


	//���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	//�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
	//���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������       
	//=============================================================================


	// ������ٶ�������ģ
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);
	norm_acc_lpf += NORM_ACC_LPF_HZ *(6.28f *half_T) *(norm_acc - norm_acc_lpf);  //10hz *3.14 * 2*0.001


	//if (ABS(ax)<8800 && ABS(ay)<8800 && ABS(az)<8800)
	if (ABS(ax)<20000 && ABS(ay)<20000 && ABS(az)<20000)
	{
		//�ѼӼƵ���ά����ת�ɵ�λ������
		ax = ax / norm_acc_lpf;//4096.0f;
		ay = ay / norm_acc_lpf;//4096.0f;
		az = az / norm_acc_lpf;//4096.0f; 

		if (15000 < norm_acc && norm_acc < 20000)
		{
			/* ��˵õ���� */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
			ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;

			/* ����ͨ */
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
	/* ������ */
	ref.err_Int.x += ref.err.x *Ki * 2 * half_T;
	ref.err_Int.y += ref.err.y *Ki * 2 * half_T;
	ref.err_Int.z += ref.err.z *Ki * 2 * half_T;

	/* �����޷� */
	ref.err_Int.x = LIMIT(ref.err_Int.x, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.y = LIMIT(ref.err_Int.y, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.z = LIMIT(ref.err_Int.z, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

	if (reference_v.z > 0.0f && ABS(* rol)<15&& ABS(* pit)<15 )
	{

		if (ctrl_s.thr>THR_BEFOR_FLY_UP )
		{
			yaw_correct = Kp *1.05f *To_180_degrees(yaw_mag - Yaw);
			//�Ѿ�������ֻ��Ҫ���پ�����
		}
		else
		{
			yaw_correct = Kp *1.05f *To_180_degrees(yaw_mag - Yaw);
			//û�н�������������ʱ�̣����پ���
		}
		// 		if( yaw_correct>360 || yaw_correct < -360  )
		// 		{
		// 			yaw_correct = 0;
		// 			//���ƾ�����Χ+-360�����+-180��ȡֵ����
		// 		}

	}
          else
          {
          yaw_correct=0;
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

	/* �ò���������PI����������ƫ */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] + (-ref_q[1] * ref.g.x - ref_q[2] * ref.g.y - ref_q[3] * ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0] * ref.g.x + ref_q[2] * ref.g.z - ref_q[3] * ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0] * ref.g.y - ref_q[1] * ref.g.z + ref_q[3] * ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0] * ref.g.z + ref_q[1] * ref.g.y - ref_q[2] * ref.g.x)*half_T;

	/* ��Ԫ����һ�� normalise quaternion */
	norm_q = my_sqrt(ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2] + ref_q[3] * ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;




	*rol = fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) *57.3f;
	*pit = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2])) *57.3f;
	// 				//Yaw   = ( - fast_atan2(2*(ref_q[1]*ref_q[2] + ref_q[0]*ref_q[3]),ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] - ref_q[3]*ref_q[3]) )* 57.3;
	*yaw = fast_atan2(2 * (-ref_q[1] * ref_q[2] - ref_q[0] * ref_q[3]), 2 * (ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1]) - 1) *57.3f;// 
	 roll_rad = *rol / 57.3;
	 pit_rad = *pit / 57.3;
	 yaw_rad = *yaw / 57.3;
	// *yaw = yaw_mag;
	//from_euler(*rol, *pit, *yaw, a_x, a_y, a_z, b_x, b_y, b_z, c_x, c_y, c_z);
     //   from_euler(*rol, *pit, 0.0, a_x, a_y, a_z, b_x, b_y, b_z, c_x, c_y, c_z);

	//from_euler((imu_ekf.roll), (imu_ekf.pith), 0.0, a_x, a_y, a_z, b_x, b_y, b_z, c_x, c_y, c_z);
//	from_euler((imu_ekf2.roll), (imu_ekf2.pith), 0.0, a_x, a_y, a_z, b_x, b_y, b_z, c_x, c_y, c_z);
	from_euler((imu_ekf2.roll_rad*57.3), (-imu_ekf2.pith_rad*57.3), 0.0, a_x, a_y, a_z, b_x, b_y, b_z, c_x, c_y, c_z);

}



void IMU_DCM::from_euler(float roll, float pitch, float yaw, float &a_x, float &a_y, float &a_z, float &b_x, float &b_y, float &b_z, float &c_x, float &c_y, float &c_z)
{
	float sinx = sinf(radians(roll));
	float cosx = cosf(radians(roll));
	float siny = sinf(radians(pitch));
	float cosy = cosf(radians(pitch));
	float sinz = sinf(radians(yaw));
	float cosz = cosf(radians(yaw));

	a_x = cosy * cosz;
	a_y = -(sinx * siny * cosz) - (cosx * sinz);
	a_z = -(cosx * siny * cosz) + (sinx * sinz);
	b_x = cosy * sinz;
	b_y = -(sinx * siny * sinz) + (cosx * cosz);
	b_z = -(cosx * siny * sinz) - (sinx * cosz);
	c_x = siny;
	c_y = sinx * cosy;
	c_z = cosx * cosy;
	//imu_ekf.C_ned2b_body(0, 0) = a_x; imu_ekf.C_ned2b_body(0,1) = a_y; imu_ekf.C_ned2b_body(0, 2) = a_z;
	//imu_ekf.C_ned2b_body(1, 0) = b_x; imu_ekf.C_ned2b_body(1, 1) = b_y; imu_ekf.C_ned2b_body(1, 2) = b_z;
	//imu_ekf.C_ned2b_body(2, 0) = c_x; imu_ekf.C_ned2b_body(2, 1) = c_y; imu_ekf.C_ned2b_body(2, 2) = c_z;

	//    float cp = arm_cos_f32(radians(pitch));
	//    float sp = arm_sin_f32(radians(pitch));
	//    float sr = arm_sin_f32(radians(roll));
	//    float cr = arm_cos_f32(radians(roll));
	//    float sy = arm_sin_f32(radians(yaw));
	//    float cy = arm_cos_f32(radians(yaw));
	//
	//    a_x = cp * cy;
	//    a_y = (sr * sp * cy) - (cr * sy);
	//    a_z = (cr * sp * cy) + (sr * sy);
	//    b_x = cp * sy;
	//    b_y = (sr * sp * sy) + (cr * cy);
	//    b_z = (cr * sp * sy) - (sr * cy);
	//    c_x = -sp;
	//    c_y = sr * cp;
	//    c_z = cr * cp;
}

