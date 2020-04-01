#include "datatrans.h"
#include "ctrl.h"
#include "AHRS.h"
#include "magnet.h"
#include "flash.h"
#include "baro.h"
#include "height_ctrl.h"
#include "gps.h"
#include "position.h"
#include "baro.h"
#include "gps_kalman_v4.h"
#include "pos_ctrl.h"
#include "imu_ekf2.h"
#include "gps_kalman_v3.h"
#include "LQR_ctrl.hpp"
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


//是否采用轮询发送
#define SEND_ONE_BETY_ONCE  0

Datatrans data_trans;


void Datatrans::send_all_fly_data_to_ANO(void)
{
	/*send_15_data((s16)ctrl_s.ctrl_2.err_d.x, (s16)ctrl_s.ctrl_1.damp.x, (s16)ctrl_s.ctrl_1.err_d.x,
	(s16)ctrl_s. ctrl_2.out.x, (s16)ahrs.Gyro_deg.y, (s16)ahrs.Gyro_deg.z,
	(s16)ctrl_s.ctrl_1.out.x, (s16)ctrl_s.ctrl_1.out.y, (s16)ctrl_s.ctrl_1.out.z,
	(float) imu_dcm.Roll, (float) imu_dcm.Pitch, (float) imu_dcm.Yaw,
	(s32) 0, (u8) 0, (u8) 0);*/

	//高度调节
	if (send_user_data_on_or_15_data == 0)
	{

		//飞控状态波形
	/*	send_15_data((s16)ctrl_s.except_AS.z, (s16)ahrs.Gyro_deg.z, (s16)(imu_ekf2.yaw_MAG),
			(s16)(ctrl_s.except_A.x*100), (s16)(pos_ctrl. y_acc_ctrl.pid_out*100), (s16)(ctrl_s.except_A.z),
			(s16)(imu_dcm.Roll), (s16)(imu_dcm.Pitch), (s16)(imu_dcm.Yaw),
		(float)(imu_ekf2.roll), (float)(imu_ekf2.pith), (float)(imu_ekf2.yaw),
			(s32)0, (u8)0, (u8)0);*/
		//飞控姿态波形
		send_15_data((s16)ctrl_s.except_AS.z, (s16)ahrs.Gyro_deg.z, (s16)(imu_ekf2.yaw_MAG),
			(s16)(ctrl_s.except_A.x * 100), (s16)(pos_ctrl.y_acc_ctrl.pid_out * 100), (s16)(imu_dcm.yaw_mag*100),
			(s16)(imu_dcm.Roll*100), (s16)(imu_dcm.Pitch*100), (s16)(imu_dcm.Yaw*100),
			(float)(imu_ekf2.roll), (float)(imu_ekf2.pith), (float)(imu_ekf2.yaw),
			(s32)0, (u8)0, (u8)0);

		send_user_data_on_or_15_data ++;
	}
	else if (send_user_data_on_or_15_data == 2)
	{
		//姿态解算波形
		//Send_User_Datas((s16)position.wx_acc, (s16)(position.wy_acc), (s16)(hlt_ctl.init_wz_acc),//1 2 3
		//	(s16)gps_kalman_v3_update.acc_x_feed_back, (s16)(-gps_kalman_v3_update.acc_y_feed_back), (s16)baro.acc_z_feed_back,           //456
		//	(s16)position.line_acc_err(0, 0), (s16)position.line_acc_err(1, 0), (s16)position.line_acc_err(2, 0),           //789
		//	(s16)ahrs.Acc.x, (s16)ahrs.Acc.y, (s16)ahrs.Acc.z,         //10 11 12
		//	(s16)gps_kalman_v3_update.x_kal_fill.X(1, 0), (s16)gps_kalman_v3_update.y_kal_fill.X(1, 0), (s16)baro.wz_kf_pos_acc,       //13 14 15
		//	(s16)(gps.gps_v_x_b*1000), (s16)(gps.gps_v_y_b*1000), (s16)baro.speed_filed,     //16 17 18
		//	(s16)17, (s16)time.time_1ms               //19 20
		//	);

		//Send_User_Datas((s16)(-imu_ekf2.line_acc(2, 0) * 1000), (s16)(hlt_ctl.wz_acc_mms2), (s16)(hlt_ctl.forward_z_speed),//1 2 3
		//	(s16)(baro.baroAlt), (s16)(baro.wz_kf_pos_acc), (s16)baro.high_kf_pos_acc,        //456
		//	(s16)(gps.vn * 1000), (s16)(gps.ve * 1000), (s16)(gps.vd * 1000),           //789
		//	//(s16)position.wx_acc, (s16)position.wy_acc, (s16)pos_ctrl.x_v_ctrl.I,         //10 11 12
		//	//	(s16)(imu_ekf2.line_acc(0, 0) * 1000), (s16)(imu_ekf2.line_acc(1, 0) * 1000), (s16)(imu_ekf2.line_acc(2, 0) * 1000),         //10 11 12
		//	(s16)( gps.px*1000), (s16)(gps.py*1000), (s16)(imu_ekf2.V_forward(2, 0) * 1000),         //10 11 12
		//	(s16)(gps_po_kf_v4.x_b), (s16)(gps_po_kf_v4.y_b), (s16)(pos_ctrl.x_pos_ctrl.err[0]),           //13 14 15
		//	(s16)(gps.gps_pos_x_b*1000), (s16)(gps.gps_pos_y_b*1000), (s16)(-imu_ekf2.V_b_lag(0, 0) * 1000),     //16 17 18
		//	(s16)(pos_ctrl.aim_position_b.x), (s16)(pos_ctrl.aim_position_b.y)             //19 20
		//	);
		
		//定点调试
		//Send_User_Datas((s16)(-imu_ekf2.V_b_forward(0, 0) * 1000), (s16)(gps.gps_v_x_b * 1000), (s16)(-imu_ekf2.V_b_lag(0, 0) * 1000),//1 2 3
		//	(s16)(-imu_ekf2.line_acc_b(0, 0) * 1000), (s16)(gps_po_kf_v4.acc_b_debias.x), (s16)(-imu_ekf2.line_acc(0, 0) * 1000),        //456
		//	(s16)(gps.px * 1000), (s16)(gps.gps_pos_x_b * 1000), (s16)(gps_po_kf_v4.x_b ),           //789
		//	//(s16)position.wx_acc, (s16)position.wy_acc, (s16)pos_ctrl.x_v_ctrl.I,         //10 11 12
		//	//	(s16)(imu_ekf2.line_acc(0, 0) * 1000), (s16)(imu_ekf2.line_acc(1, 0) * 1000), (s16)(imu_ekf2.line_acc(2, 0) * 1000),         //10 11 12
		//	(s16)(pos_ctrl.aim_position_b.x), (s16)(pos_ctrl.aim_v.x), (s16)(pos_ctrl.aim_acc.x),         //10 11 12
		//	(s16)(gps_po_kf_v4.acc_b_debias.x  ), (s16)(gps_po_kf_v4.acc_b_debias.y ), (s16)(-imu_ekf2.line_acc_b(0, 0) * 1000),          //13 14 15
		//	(s16)(gps.gps_pos_y_b * 1000), (s16)(gps.gps_v_x_b*1000), (s16)(hlt_ctl.exp_height),     //16 17 18
		//	(s16)(baro.baroAlt), (s16)(baro.high_kf_pos_acc)             //19 20
		//	);
		//系统辨识
		//Send_User_Datas((s16)(-imu_ekf2.V_b_forward(0, 0) * 1000), (s16)(imu_ekf2.V_b_forward(1, 0) * 1000), (s16)(imu_ekf2.V_b_forward(2, 0) * 1000),//1 2 3
		//	(s16)(-imu_ekf2.line_acc_b(0, 0) * 1000), (s16)(imu_ekf2.line_acc_b(1, 0) * 1000), (s16)(imu_ekf2.line_acc_b(2, 0) * 1000),        //456
		//	(s16)(gps.gps_pos_x_b * 1000), (s16)(gps.gps_pos_y_b * 1000), (s16)(gps_po_kf_v4.x_b),           //789
		//	//(s16)position.wx_acc, (s16)position.wy_acc, (s16)pos_ctrl.x_v_ctrl.I,         //10 11 12
		//	//	(s16)(imu_ekf2.line_acc(0, 0) * 1000), (s16)(imu_ekf2.line_acc(1, 0) * 1000), (s16)(imu_ekf2.line_acc(2, 0) * 1000),         //10 11 12
		//	(s16)(pos_ctrl.aim_position_b.x), (s16)(pos_ctrl.aim_v.x), (s16)(pos_ctrl.aim_acc.x),         //10 11 12
		//	(s16)(pos_ctrl.aim_position_b.y), (s16)(pos_ctrl.aim_v.y), (s16)(pos_ctrl.aim_acc.y),            //13 14 15
		//	(s16)(gps.gps_pos_y_b * 1000), (s16)(gps.gps_v_x_b * 1000), (s16)(hlt_ctl. height_ctrl_out),     //16 17 18
		//	(s16)(baro.baroAlt), (s16)(baro.high_kf_pos_acc)             //19 20
		//	);
		//控制器仿真
		Send_User_Datas((s16)(simulink.aim_speed), (s16)(simulink.real_speed), (s16)(pos_ctrl.x_v_ctrl.pid_out),//1 2 3
			(s16)(simulink.aim_pos), (s16)(simulink.real_pos), (s16)(imu_ekf2.line_acc_b(2, 0) * 1000),        //456
			(s16)(gps.gps_pos_x_b * 1000), (s16)(gps.gps_pos_y_b * 1000), (s16)(gps_po_kf_v4.x_b),           //789
			//(s16)position.wx_acc, (s16)position.wy_acc, (s16)pos_ctrl.x_v_ctrl.I,         //10 11 12
			//	(s16)(imu_ekf2.line_acc(0, 0) * 1000), (s16)(imu_ekf2.line_acc(1, 0) * 1000), (s16)(imu_ekf2.line_acc(2, 0) * 1000),         //10 11 12
			(s16)(pos_ctrl.aim_position_b.x), (s16)(pos_ctrl.aim_v.x), (s16)(pos_ctrl.aim_acc.x),         //10 11 12
			(s16)(pos_ctrl.aim_position_b.y), (s16)(pos_ctrl.aim_v.y), (s16)(pos_ctrl.aim_acc.y),            //13 14 15
			(s16)(gps.gps_pos_y_b * 1000), (s16)(gps.gps_v_x_b * 1000), (s16)(hlt_ctl.height_ctrl_out),     //16 17 18
			(s16)(baro.baroAlt), (s16)(baro.high_kf_pos_acc)             //19 20
			);
		//experimental data1(initial可以用于matlab进行仿真的原始数据)
		//Send_User_Datas((s16)ahrs.Gyro_deg.x, (s16)(ahrs.Gyro_deg.y), (s16)(ahrs.Gyro_deg.z),//1 2 3
		//	(s16)ahrs.Acc.x, (s16)ahrs.Acc.y, (s16)ahrs.Acc.z,           //456
		//	(s16)mag_s.Mag_Val.x, (s16)mag_s.Mag_Val.y, (s16)mag_s.Mag_Val.z,           //789
		//	(s16)(gps.px * 1000), (s16)(gps.py * 1000), (s16)(gps.pz),         //10 11 12
		//	(s16)(gps.vn * 1000), (s16)(gps.ve * 1000), (s16)(gps.vd * 1000), //13 14 15
		//	(s16)(gps.gps_pos_x_b * 1000), (s16)(gps.gps_pos_y_b * 1000), (s16)(gps.gps_v_x_b * 1000),//16 17 18
		//	(s16)(gps.gps_v_y_b * 1000), (s16)time.time_1ms               //19 20
		//	);
		//experimental data1(原始数据)
		//Send_User_Datas((s16)ahrs.Gyro_deg.x, (s16)(ahrs.Gyro_deg.y), (s16)(ahrs.Gyro_deg.z),//1 2 3
		//	(s16)ahrs.Acc.x, (s16)ahrs.Acc.y, (s16)ahrs.Acc.z,           //456
		//	(s16)mag_s.Mag_Val.x, (s16)mag_s.Mag_Val.y, (s16)mag_s.Mag_Val.z,           //789
		//	(s16)(gps.px * 1000), (s16)(gps.py * 1000), (s16)(gps.pz),         //10 11 12
		//	(s16)(gps.vn * 1000), (s16)(gps.ve * 1000), (s16)(gps.vd * 1000), //13 14 15
		//	(s16)(gps.gps_pos_x_b * 1000), (s16)(gps.gps_pos_y_b * 1000), (s16)(gps.gps_v_x_b * 1000),//16 17 18
		//	(s16)(gps.gps_v_y_b * 1000), (s16)time.time_1ms               //19 20
		//	);
		//experimental data1(融合结果)
		//Send_User_Datas(
		//	(s16)(-imu_ekf2.V_b_forward(0, 0) * 1000), (s16)(imu_ekf2.V_b_forward(1, 0) * 1000), (s16)(-imu_ekf2.V_b_forward(2, 0) * 1000),//1 2 3
		//	(s16)(-imu_ekf2.V_b_lag(0, 0) * 1000), (s16)(imu_ekf2.V_b_lag(1, 0) * 1000), (s16)(-imu_ekf2.V_b_lag(2, 0) * 1000),     //456
		//	(s16)(-imu_ekf2.line_acc_b(0, 0) * 1000), (s16)(-imu_ekf2.line_acc_b(1, 0) * 1000), (s16)(-imu_ekf2.line_acc_b(2, 0) * 1000),           //789
		//	(s16)(gps.px * 1000), (s16)(gps.py * 1000), (s16)(gps.pz),         //10 11 12
		//	(s16)(gps.vn * 1000), (s16)(gps.ve * 1000), (s16)(gps.vd * 1000), //13 14 15
		//	(s16)(gps.gps_pos_x_b * 1000), (s16)(gps.gps_pos_y_b * 1000), (s16)(gps.gps_v_x_b * 1000),//16 17 18
		//	(s16)(gps.gps_v_y_b * 1000), (s16)time.time_1ms               //19 20
		//	);

		//Send_User_Datas((s16)(-imu_ekf2.V_b_forward(0, 0) * 1000), (s16)(imu_ekf2.V_b_forward(1, 0) * 1000), (s16)(imu_ekf2.V_b_lag(1, 0) * 1000),//1 2 3
		//(s16)(-imu_ekf2.line_acc_b(0, 0) * 1000), (s16)(imu_ekf2.line_acc_b(1, 0) * 1000), (s16)(-imu_ekf2.line_acc(0, 0) * 1000),        //456
		//	(s16)(gps.px * 1000), (s16)(gps.gps_pos_x_b * 1000), (s16)(gps_po_kf_v4.x_b),           //789
		//	//(s16)position.wx_acc, (s16)position.wy_acc, (s16)pos_ctrl.x_v_ctrl.I,         //10 11 12
		//	//	(s16)(imu_ekf2.line_acc(0, 0) * 1000), (s16)(imu_ekf2.line_acc(1, 0) * 1000), (s16)(imu_ekf2.line_acc(2, 0) * 1000),         //10 11 12
		//	(s16)(imu_ekf2.gps_fix_err.x * 1000), (s16)(imu_ekf2.gps_fix_err.y * 1000), (s16)(imu_ekf2.gps_fix_err.z * 1000),         //10 11 12
		//	(s16)(gps.vn * 1000), (s16)(gps.ve * 1000), (s16)(gps.vd * 1000),          //13 14 15
		//	(s16)(imu_ekf2.ini_fixed_speed.x * 1000), (s16)(imu_ekf2.ini_fixed_speed.y * 1000), (s16)(imu_ekf2.ini_fixed_speed.z * 1000),     //16 17 18
		//	(s16)(gps.gps_v_x_b * 1000), (s16)(gps.gps_v_y_b * 1000)  //19 20;
		//	);         
		//原始数据
		//Send_User_Datas((s16)ahrs.Gyro_deg.x, (s16)(ahrs.Gyro_deg.y), (s16)(ahrs.Gyro_deg.z),//1 2 3
		//	(s16)ahrs.Acc.x, (s16)ahrs.Acc.y, (s16)ahrs.Acc.z,           //456
		//	(s16)mag_s.Mag_Val.x, (s16)mag_s.Mag_Val.y, (s16)mag_s.Mag_Val.z,           //789
		//	(s16)(mag_s.Mag_Adc.x), (s16)(mag_s.Mag_Adc.y), (s16)(mag_s.Mag_Adc.z),         //10 11 12
		//	(s16)(gps.gps_v_x_b * 1000), (s16)(gps.gps_v_y_b * 1000), (s16)(gps.vd * 1000), //13 14 15
		//	(s16)imu_dcm.Roll, (s16)imu_dcm.Pitch, (s16)imu_dcm.Yaw,//16 17 18
		//	(s16)17, (s16)time.time_1ms               //19 20
		//	);
		////ekf2调试相关
		//Send_User_Datas((s16)(-imu_ekf2.V_b_lag(0, 0) * 1000), (s16)(imu_ekf2.V_b_lag(1, 0) * 1000), (s16)(imu_ekf2.V_b_lag(2, 0) * 1000),//1 2 3
		//	(s16)(baro.baroAlt), (s16)(baro.wz_kf_pos_acc), (s16)baro.high_kf_pos_acc,        //456
		//	(s16)(gps.vn * 1000), (s16)(gps.ve * 1000), (s16)(gps.vd*1000),           //789
		//	//(s16)position.wx_acc, (s16)position.wy_acc, (s16)pos_ctrl.x_v_ctrl.I,         //10 11 12
		////	(s16)(imu_ekf2.line_acc(0, 0) * 1000), (s16)(imu_ekf2.line_acc(1, 0) * 1000), (s16)(imu_ekf2.line_acc(2, 0) * 1000),         //10 11 12
		//(s16)(-imu_ekf2.V_forward(0, 0) * 1000), (s16)(imu_ekf2.V_forward(1, 0) * 1000), (s16)(imu_ekf2.V_forward(2, 0) * 1000),         //10 11 12
		//(s16)(-imu_ekf2.V_b_forward(0, 0) * 1000), (s16)(imu_ekf2.V_b_forward(1, 0) * 1000), (s16)(imu_ekf2.V_b_forward(2, 0) * 1000),           //13 14 15
		//	(s16)(-imu_ekf2.xhat_k(4, 0) * 1000), (s16)(imu_ekf2.xhat_k(5, 0) * 1000), (s16)(-imu_ekf2.xhat_k(6, 0) * 1000),     //16 17 18
		//	(s16)(gps.gps_v_x_b * 1000), (s16)(gps.gps_v_y_b * 1000)             //19 20
		// 		);
		////定高调试相关
		////Send_User_Datas((s16)(baro.baroAlt), (s16)(baro.baro_alt_speed), (s16)baro.high_kf_pos_acc,//1 2 3
		////	(s16)baro.wz_kf_pos_acc, (s16)hlt_ctl.forward_z_speed, (s16)hlt_ctl.wz_acc,           //456
		////	(s16)hlt_ctl.wz_acc_old[99], (s16)(hlt_ctl.ultra_ctrl.err), (s16)hlt_ctl. ultra_ctrl.pid_out,           //789
		////	(s16)hlt_ctl.ultra_ctrl.err_i, (s16)(10*hlt_ctl.ultra_ctrl.err_d), (s16)hlt_ctl. wz_speed_pid_v.pid_out_err,         //10 11 12
		////	(s16)hlt_ctl.wz_speed_pid_v.err_i, (s16)(10 * hlt_ctl.wz_speed_pid_v.err_d), (s16)hlt_ctl.exp_height,       //13 14 15
		////	(s16)(ctrl_s.thr_value), (s16)(imu_ekf2.xhat_k(5, 0) * 1000), (s16)gps.vn,     //16 17 18
		////	(s16)(gps.vd*1000), (s16)position.wx_acc_ini              //19 20
		////	);

	//	定高调试相关模糊
		//Send_User_Datas((s16)(baro.baroAlt), (s16)(hlt_ctl. ultra_ctrl.pid_out), (s16)baro.high_kf_pos_acc,//1 2 3
		//	(s16)hlt_ctl.wz_acc_mms2, (s16)hlt_ctl.forward_z_speed, (s16)hlt_ctl.ultra_ctrl_out,           //456
		//	(s16)hlt_ctl. acc_pid_v.pid_out_err, (s16)(hlt_ctl.ultra_ctrl.err), (s16)hlt_ctl.height_ctrl_out,           //789
		//	(s16)hlt_ctl.ultra_ctrl.err_i, (s16)(10*hlt_ctl.ultra_ctrl.err_d), (s16)hlt_ctl. wz_speed_pid_v.pid_out_err,         //10 11 12
		//	(s16)hlt_ctl.wz_speed_pid_v.err_i, (s16)(10 * hlt_ctl.wz_speed_pid_v.err_d), (s16)hlt_ctl.exp_height,       //13 14 15
		//	(s16)(ctrl_s.thr_value), (s16)(imu_ekf2.xhat_k(5, 0) * 1000), (s16)gps.vn,     //16 17 18
		//	(s16)(gps.vd*1000), (s16)(hlt_ctl.fuzzy_kp_weight*1000)              //19 20
		//	);
		send_user_data_on_or_15_data++;
	}
	else
	{
		send_user_data_on_or_15_data++;
		if (send_user_data_on_or_15_data==4)
		{
			send_user_data_on_or_15_data = 0;
		}
	}

	 

}
void Datatrans::acopter_Send_Data(u8 *dataToSend, u8 length)
{
	HAL_UART_Transmit_DMA(&huart2, dataToSend, length);
	//HAL_UART_Transmit(&huart2, dataToSend, length,101);
}

void Datatrans::Send_User_Datas(s16 user1, s16 user2, s16 user3,
	s16 user4, s16 user5, s16 user6,
	s16 user7, s16 user8, s16 user9,
	s16 user10, s16 user11, s16 user12,
	s16 user13, s16 user14, s16 user15,
	s16 user16, s16 user17, s16 user18,
	s16 user19, s16 user20
	)
{


	static short send_all_usr_data_cnt;
	static short cnt_usr = 0;
	short _temp;

	if (send_all_usr_data == 0)
	{

		cnt_usr = 0;
		user_data_to_send[cnt_usr++] = 0xAA;
		user_data_to_send[cnt_usr++] = 0xAA;
		user_data_to_send[cnt_usr++] = 0xf1; //用户数据
		user_data_to_send[cnt_usr++] = 0;


		_temp = (s16)user1;            //1
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);

		_temp = (s16)user2;
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);

		_temp = (s16)user3;
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);

		_temp = (s16)user4;
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);

		_temp = (s16)user5;              //5
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user6;              //6
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user7;              //7
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user8;              //8
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user9;              //9
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user10;              //10
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user11;              //11
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);

		_temp = (s16)user12;              //12
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user13;              //13
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user14;              //14
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user15;              //15
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user16;              //16
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user17;              //17
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user18;              //18
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user19;              //19
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);
		_temp = (s16)user20;              //20
		user_data_to_send[cnt_usr++] = BYTE1(_temp);
		user_data_to_send[cnt_usr++] = BYTE0(_temp);

		user_data_to_send[3] = cnt_usr - 4;
		u8 sum = 0;
		for (u8 i = 0; i < cnt_usr; i++)
			sum += user_data_to_send[i];
		user_data_to_send[cnt_usr++] = sum;

		send_all_usr_data = 1;
		send_all_usr_data_cnt = 0;
	}
#if  SEND_ONE_BETY_ONCE
	if (send_all_usr_data == 1)
	{
		acopter_Send_Data(&user_data_to_send[send_all_usr_data_cnt], (u8)1);
		send_all_usr_data_cnt++;
		if (send_all_usr_data_cnt > (cnt_usr + 1))
		{
			send_all_usr_data = 0;

		}
	}
#else
	acopter_Send_Data(user_data_to_send, (u8)cnt_usr);

	send_all_usr_data_cnt = 0;
	send_all_usr_data = 0;
#endif

}

void Datatrans::send_15_data(s16 a_x, s16 a_y, s16 a_z,
	s16 g_x, s16 g_y, s16 g_z,
	s16 m_x, s16 m_y, s16 m_z,
	float angle_rol, float angle_pit, float angle_yaw,
	s32 alt, u8 fly_model, u8 armed)
{
	static u8 send_all_data = 0;
	static u8 send_all_data_cnt = 0;
	static u8 _cnt = 0;
	if (send_all_data == 0)
	{
		_cnt = 0;
		vs16 _temp;

		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0x02;
		data_to_send[_cnt++] = 0;

		_temp = a_x;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = a_y;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = a_z;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		_temp = g_x;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = g_y;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = g_z;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		_temp = m_x;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = m_y;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = m_z;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		/////////////////////////////////////////
		_temp = 0;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		data_to_send[3] = _cnt - 4;

		u8 sum = 0;
		for (u8 i = 0; i<_cnt; i++)
			sum += data_to_send[i];
		data_to_send[_cnt++] = sum;


		u8 _cnt2 = 0;
		u8 _cnt3 = 0;
		int32_t _temp2 = alt;
		_cnt3 = _cnt;
		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0x01;
		_cnt2 = _cnt;
		data_to_send[_cnt++] = 0;

		_temp = (int)(angle_rol * 100);
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = (int)(angle_pit * 100);
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = (int)(angle_yaw * 100);
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		data_to_send[_cnt++] = BYTE3(_temp2);
		data_to_send[_cnt++] = BYTE2(_temp2);
		data_to_send[_cnt++] = BYTE1(_temp2);
		data_to_send[_cnt++] = BYTE0(_temp2);

		data_to_send[_cnt++] = fly_model;

		data_to_send[_cnt++] = armed;

		data_to_send[_cnt2] = _cnt - _cnt2 - 1;

		sum = 0;
		for (u8 i = _cnt3; i<_cnt; i++)
			sum += data_to_send[i];
		data_to_send[_cnt++] = sum;


		send_all_data = 1;
		send_all_data_cnt = 0;
	}
#if  SEND_ONE_BETY_ONCE
	if (send_all_data == 1)
	{
		acopter_Send_Data(&data_to_send[send_all_data_cnt], (u8)1);
		send_all_data_cnt++;
		if (send_all_data_cnt>_cnt)
		{

			send_all_data = 0;
		}
	}
#else
	acopter_Send_Data(data_to_send, (u8)_cnt);

	send_all_data = 0;

#endif


}
void Datatrans::ANO_DT_Send_PID(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
	u8 _cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x10 + group - 1;
	data_to_send[_cnt++] = 0;


	_temp = (short)(p1_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p1_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p1_d * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_d * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_d * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;

	u8 sum = 0;
	for (u8 i = 0; i<_cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;

	acopter_Send_Data(data_to_send, _cnt);
}
void  Datatrans::data_trans_with_ano()
{

	/*---------------参数改变---------------*/
	if (get_data_from_ANO == 1)
	{

		ANO_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5);
		get_data_from_ANO = 0;
		return;
	}

	/*---------------返回参数---------------*/
	if (send_pid_para)
	{
		if (send_pid_para == 1)
		{

			ANO_DT_Send_PID(1, ctrl_s.ctrl_1.PID[PIDROLL].kp, ctrl_s.ctrl_1.PID[PIDROLL].ki, ctrl_s.ctrl_1.PID[PIDROLL].kd,
				ctrl_s.ctrl_1.PID[PIDPITCH].kp, ctrl_s.ctrl_1.PID[PIDPITCH].ki, ctrl_s.ctrl_1.PID[PIDPITCH].kd,
				ctrl_s.ctrl_1.PID[PIDYAW].kp, ctrl_s.ctrl_1.PID[PIDYAW].ki, ctrl_s.ctrl_1.PID[PIDYAW].kd);
			send_pid_para++;
			return;
		}
		else if (send_pid_para == 20)
		{


			ANO_DT_Send_PID(2, ctrl_s.ctrl_2.PID[PIDROLL].kp, ctrl_s.ctrl_2.PID[PIDROLL].ki, ctrl_s.ctrl_2.PID[PIDROLL].kd,
				ctrl_s.ctrl_2.PID[PIDPITCH].kp, ctrl_s.ctrl_2.PID[PIDPITCH].ki, ctrl_s.ctrl_2.PID[PIDPITCH].kd,
				ctrl_s.ctrl_2.PID[PIDYAW].kp, ctrl_s.ctrl_2.PID[PIDYAW].ki, ctrl_s.ctrl_2.PID[PIDYAW].kd);
			send_pid_para++;
			return;

		}
		else if (send_pid_para == 30)
		{


			ANO_DT_Send_PID(3, hlt_ctl.ultra_pid.kp, hlt_ctl.ultra_pid.ki, hlt_ctl.ultra_pid.kd,
				hlt_ctl.wz_speed_pid.kp, hlt_ctl.wz_speed_pid.ki, hlt_ctl.wz_speed_pid.kd,
				//位置外环33
				pos_ctrl.x_pos_ctrl.Kp, pos_ctrl.x_pos_ctrl.Ki, pos_ctrl.x_pos_ctrl.Kd);
			send_pid_para++;
			return;

		}
		else if (send_pid_para == 40)
		{
			//位置内环41
			ANO_DT_Send_PID(4, pos_ctrl.x_v_ctrl.Kp, pos_ctrl.x_v_ctrl.Ki, pos_ctrl.x_v_ctrl.Kd,
				hlt_ctl.chang_fuzzy, hlt_ctl.max_kp_weight, hlt_ctl.kp_weight_diff_point,  //高度分段参数
				pos_ctrl.x_pos_ctrl.kp_weight_amplify, 
				pos_ctrl.x_pos_ctrl.max_kp_weight, 
				pos_ctrl.x_pos_ctrl.kp_weight_diff_point);  //位置控制分段参数
			send_pid_para++;
			return;

		}
		else if (send_pid_para == 50)
		{


			ANO_DT_Send_PID(5, hlt_ctl.acc_pid.kp, hlt_ctl.acc_pid.ki, hlt_ctl.acc_pid.kd,//高度加速度换
				pos_ctrl.x_acc_ctrl.Kp, pos_ctrl.x_acc_ctrl.Ki, pos_ctrl.x_acc_ctrl.Kd,//位置加速度环参数
				0, ctrl_s.ctrl_2.PID[PIDYAW].ki, pos_ctrl.x_pos_ctrl.chang_pos_fuzzy);
			send_pid_para++;
			return;

		}
		else if (send_pid_para == 60)
		{


			ANO_DT_Send_PID(6, ctrl_s.ctrl_2.PID[PIDROLL].kp, ctrl_s.ctrl_2.PID[PIDROLL].ki, ctrl_s.ctrl_2.PID[PIDROLL].kd,
				ctrl_s.ctrl_2.PID[PIDPITCH].kp, ctrl_s.ctrl_2.PID[PIDPITCH].ki, ctrl_s.ctrl_2.PID[PIDPITCH].kd,
				ctrl_s.ctrl_2.PID[PIDYAW].kp, ctrl_s.ctrl_2.PID[PIDYAW].ki, ctrl_s.ctrl_2.PID[PIDYAW].kd);
			send_pid_para = 0;
			return;

		}

		send_pid_para++;
		return;
	}

	/*---------------波形发送---------------*/
	send_all_fly_data_to_ANO();


}

void Datatrans::ANO_data_receive_prepara(void)
{
	u8 data;

	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET)
	{
		data = huart2.Instance->DR;
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);

		if (state == 0 && data == 0xAA)
		{
			state = 1;
			RxBuffer[0] = data;
		}
		else if (state == 1 && data == 0xAF)
		{
			state = 2;
			RxBuffer[1] = data;
		}
		else if (state == 2 && data<0XF1)
			//else if(state==2&&(data==0X11))
		{
			state = 3;
			RxBuffer[2] = data;
		}
		else if (state == 3 && data<50)
		{
			state = 4;
			RxBuffer[3] = data;
			_data_len = data;
			_data_cnt = 0;
		}
		else if (state == 4 && _data_len>0)
		{
			_data_len--;
			RxBuffer[4 + _data_cnt++] = data;
			if (_data_len == 0)
				state = 5;
		}
		else if (state == 5)
		{
			state = 0;
			RxBuffer[4 + _data_cnt] = data;
			get_data_from_ANO = 1;
		}
		else
			state = 0;


	}

}

void Datatrans::ANO_DT_Data_Receive_Anl(u8 *data_buf, u8 num)
{
	u8 sum = 0;
	for (u8 i = 0; i<(num - 1); i++)
		sum += *(data_buf + i);
	if (!(sum == *(data_buf + num - 1)))		return;		//判断sum
	if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))		return;		//判断帧头

	/*------传感器校准------*/
	if (*(data_buf + 2) == 0X01)
	{
		if (*(data_buf + 4) == 0X01)
		{
			ahrs.Acc_CALIBRATE = 1;
			//mpu6050.Cali_3d = 1;
		}
		else if (*(data_buf + 4) == 0X02)
			ahrs.Gyro_CALIBRATE = 1;
		/*else if (*(data_buf + 4) == 0X03)
		{
		mpu6050.Acc_CALIBRATE = 1;
		mpu6050.Gyro_CALIBRATE = 1;
		}*/
		else if (*(data_buf + 4) == 0X04)
		{
			if (mag_s.Mag_CALIBRATED ==0)
			{
				mag_s.Mag_CALIBRATED = 1;
			}
			else if (mag_s.Mag_CALIBRATED == 1)
			{
				mag_s.Mag_CALIBRATED = 0;
			}
		 
		}

	}


	if (*(data_buf + 2) == 0X02)
	{
		//返回参数
		if (*(data_buf + 4) == 0X01)
		{
			send_pid_para = 1;

		}

		if (*(data_buf + 4) == 0XA1)
		{


			flash_save.save_on = 1;


		}


	}



	if (*(data_buf + 2) == 0X10)								//PID1
	{



		ctrl_s.ctrl_1.PID[PIDROLL].kp = 0.001*((short)(*(data_buf + 4) << 8) | *(data_buf + 5));
		ctrl_s.ctrl_1.PID[PIDROLL].ki = 0.001*((short)(*(data_buf + 6) << 8) | *(data_buf + 7));
		ctrl_s.ctrl_1.PID[PIDROLL].kd = 0.001*((short)(*(data_buf + 8) << 8) | *(data_buf + 9));

		ctrl_s.ctrl_1.PID[PIDPITCH].kp = 0.001*((short)(*(data_buf + 10) << 8) | *(data_buf + 11));
		ctrl_s.ctrl_1.PID[PIDPITCH].ki = 0.001*((short)(*(data_buf + 12) << 8) | *(data_buf + 13));
		ctrl_s.ctrl_1.PID[PIDPITCH].kd = 0.001*((short)(*(data_buf + 14) << 8) | *(data_buf + 15));




		ctrl_s.ctrl_1.PID[PIDYAW].kp = 0.001*((short)(*(data_buf + 16) << 8) | *(data_buf + 17));
		ctrl_s.ctrl_1.PID[PIDYAW].ki = 0.001*((short)(*(data_buf + 18) << 8) | *(data_buf + 19));
		ctrl_s.ctrl_1.PID[PIDYAW].kd = 0.001*((short)(*(data_buf + 20) << 8) | *(data_buf + 21));


		ANO_DT_Send_Check(*(data_buf + 2), sum);
		//Param_SavePID();
		//Ctrl_Para_Init();
		//	flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X11)								//PID2
	{
		ctrl_s.ctrl_2.PID[PIDROLL].kp = 0.001*((short)(*(data_buf + 4) << 8) | *(data_buf + 5));
		ctrl_s.ctrl_2.PID[PIDROLL].ki = 0.001*((short)(*(data_buf + 6) << 8) | *(data_buf + 7));
		ctrl_s.ctrl_2.PID[PIDROLL].kd = 0.001*((short)(*(data_buf + 8) << 8) | *(data_buf + 9));
		ctrl_s.ctrl_2.PID[PIDPITCH].kp = 0.001*((short)(*(data_buf + 10) << 8) | *(data_buf + 11));
		ctrl_s.ctrl_2.PID[PIDPITCH].ki = 0.001*((short)(*(data_buf + 12) << 8) | *(data_buf + 13));
		ctrl_s.ctrl_2.PID[PIDPITCH].kd = 0.001*((short)(*(data_buf + 14) << 8) | *(data_buf + 15));
		ctrl_s.ctrl_2.PID[PIDYAW].kp = 0.001*((short)(*(data_buf + 16) << 8) | *(data_buf + 17));
		ctrl_s.ctrl_2.PID[PIDYAW].ki = 0.001*((short)(*(data_buf + 18) << 8) | *(data_buf + 19));
		ctrl_s.ctrl_2.PID[PIDYAW].kd = 0.001*((short)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
		//Ctrl_Para_Init();
		//flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X12)								//PID3
	{
		hlt_ctl.ultra_pid.kp = 0.001*((short)(*(data_buf + 4) << 8) | *(data_buf + 5));
		hlt_ctl.ultra_pid.ki = 0.001*((short)(*(data_buf + 6) << 8) | *(data_buf + 7));
		hlt_ctl.ultra_pid.kd = 0.001*((short)(*(data_buf + 8) << 8) | *(data_buf + 9));
		hlt_ctl.wz_speed_pid.kp = 0.001*((short)(*(data_buf + 10) << 8) | *(data_buf + 11));
		hlt_ctl.wz_speed_pid.ki = 0.001*((short)(*(data_buf + 12) << 8) | *(data_buf + 13));
		hlt_ctl.wz_speed_pid.kd = 0.001*((short)(*(data_buf + 14) << 8) | *(data_buf + 15));

		//位置外化33
		pos_ctrl.x_pos_ctrl.Kp = 0.001*((short)(*(data_buf + 16) << 8) | *(data_buf + 17));
		pos_ctrl.y_pos_ctrl.Kp = pos_ctrl.x_pos_ctrl.Kp;
		pos_ctrl.x_pos_ctrl.Ki = 0.001*((short)(*(data_buf + 18) << 8) | *(data_buf + 19));
		pos_ctrl.y_pos_ctrl.Ki = pos_ctrl.x_pos_ctrl.Ki;
		pos_ctrl.x_pos_ctrl.Kd = 0.001* ((short)(*(data_buf + 20) << 8) | *(data_buf + 21));
		pos_ctrl.y_pos_ctrl.Kd = pos_ctrl.x_pos_ctrl.Kd;

		ANO_DT_Send_Check(*(data_buf + 2), sum);
		//PID_Para_Init();
		//flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X13)								//PID4
	{
		//位置内化41
		pos_ctrl.x_v_ctrl.Kp = 0.001* ((short)(*(data_buf + 4) << 8) | *(data_buf + 5));
		pos_ctrl.y_v_ctrl.Kp = pos_ctrl.x_v_ctrl.Kp;
		pos_ctrl.x_v_ctrl.Ki = 0.001*((short)(*(data_buf + 6) << 8) | *(data_buf + 7));
		pos_ctrl.y_v_ctrl.Ki = pos_ctrl.x_v_ctrl.Ki;
		pos_ctrl.x_v_ctrl.Kd = 0.001*((short)(*(data_buf + 8) << 8) | *(data_buf + 9));
		pos_ctrl.y_v_ctrl.Kd = pos_ctrl.x_v_ctrl.Kd;
		//高度分段控制参数
		hlt_ctl.chang_fuzzy = 0.001*((short)(*(data_buf + 10) << 8) | *(data_buf + 11));
		hlt_ctl.max_kp_weight= 0.001*((short)(*(data_buf + 12) << 8) | *(data_buf + 13));
		hlt_ctl.kp_weight_diff_point= 0.001*((short)(*(data_buf + 14) << 8) | *(data_buf + 15));
		//位置分段
		pos_ctrl.x_pos_ctrl.kp_weight_amplify = 0.001*((short)(*(data_buf + 16) << 8) | *(data_buf + 17));
		pos_ctrl.y_pos_ctrl.kp_weight_amplify = pos_ctrl.x_pos_ctrl.kp_weight_amplify;
		pos_ctrl.x_pos_ctrl.max_kp_weight = 0.001*((short)(*(data_buf + 18) << 8) | *(data_buf + 19));
		pos_ctrl.y_pos_ctrl.max_kp_weight = pos_ctrl.x_pos_ctrl.max_kp_weight;
		pos_ctrl.x_pos_ctrl.kp_weight_diff_point = 0.001* ((short)(*(data_buf + 20) << 8) | *(data_buf + 21));
		pos_ctrl.y_pos_ctrl.kp_weight_diff_point = pos_ctrl.x_pos_ctrl.kp_weight_diff_point;

		ANO_DT_Send_Check(*(data_buf + 2), sum);

		//PID_Para_Init();
		//flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X14)								//PID5
	{
		//高度加速度环
		hlt_ctl.acc_pid.kp = 0.001*((short)(*(data_buf + 4) << 8) | *(data_buf + 5));
		hlt_ctl.acc_pid.ki = 0.001*((short)(*(data_buf + 6) << 8) | *(data_buf + 7));
		hlt_ctl.acc_pid.kd = 0.001*((short)(*(data_buf + 8) << 8) | *(data_buf + 9));
		//位置加速度环
		pos_ctrl.x_acc_ctrl.Kp = 0.001*((short)(*(data_buf + 10) << 8) | *(data_buf + 11));
		pos_ctrl.x_acc_ctrl.Ki = 0.001*((short)(*(data_buf + 12) << 8) | *(data_buf + 13));
		pos_ctrl.x_acc_ctrl.Kd = 0.001*((short)(*(data_buf + 14) << 8) | *(data_buf + 15));
		pos_ctrl.y_acc_ctrl.Kp = pos_ctrl.x_acc_ctrl.Kp;
		pos_ctrl.y_acc_ctrl.Ki = pos_ctrl.x_acc_ctrl.Ki;
		pos_ctrl.y_acc_ctrl.Kd = pos_ctrl.x_acc_ctrl.Kd;

		pos_ctrl.x_pos_ctrl.chang_pos_fuzzy = 0.001* ((short)(*(data_buf + 20) << 8) | *(data_buf + 21));
		pos_ctrl.y_pos_ctrl.chang_pos_fuzzy = pos_ctrl.x_pos_ctrl.chang_pos_fuzzy;
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	if (*(data_buf + 2) == 0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}


}

void Datatrans::ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0] = 0xAA;
	data_to_send[1] = 0xAA;
	data_to_send[2] = 0xEF;
	data_to_send[3] = 2;
	data_to_send[4] = head;
	data_to_send[5] = check_sum;


	u8 sum = 0;
	for (u8 i = 0; i<6; i++)
		sum += data_to_send[i];
	data_to_send[6] = sum;

	acopter_Send_Data(data_to_send, 7);
}