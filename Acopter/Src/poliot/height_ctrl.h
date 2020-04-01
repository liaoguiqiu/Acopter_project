#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "include.h"
#include "fuzzy.h"
#define ULTRA_SPEED 		 300    // mm/s

#define ULTRA_INT        600    // ????
#define BARO_SPEED_NUM 100
#define EXP_Z_SPEED  ( 4.0f *my_deathzoom( (thr-450), 50 ) )
#define THR_BEFOR_FLY_UP  350



typedef struct
{
	float err;
	float err_old;
	float err_d;
	float err_d_save[20];
	float err_i;
	float pid_out;
	float pid_out_err;
}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;

}_st_height_pid;

typedef struct 
{

	short fly_up_cnt;
	u8 fly_up;
	short fly_down_cnt;

} FLY_UP_DOWN;

class HLT_CTRL
{
public:
	HLT_CTRL()
	{
		acc_pid.kp = 0.015;
		acc_pid.ki = 0.05;
		acc_pid.kd = 0.030;
		acc_death = 150;
		wz_death = 0;
		z_height_ki = 0.1;
		z_height_ki_2 = 0.01;
		height_ki = 0.002;
		speed_lp_hz2 = 0.1;
		z_height_loop_hz1 = 0.2;

		ULTRA_MAX_HEIGHT = 6000;
		gps_ki = 0.1;
                speed_lp_test = 0.6;
				kp_weight_amplify = 2;
				max_kp_weight = 2;
				kp_weight_diff_point = 0.6;
				chang_fuzzy = 1.63;
	}
	float chang_fuzzy;
	  float thr_lpf;
	Fuzzy_PID  fuzzy_KP;
	float fuzzy_kp_weight;
	unsigned char outer_ctrl_wait_cnt;
	float kp_weight_diff_point; //m
	float kp_weight_amplify;//kp放大系数
	float max_kp_weight;  //kp放大限幅
         float speed_lp_test ;
	float z_height_gps;
	float gps_ki;
	float gps_i;
	short ULTRA_MAX_HEIGHT; // mm  
	float aim_height;
	float baro_speed_arr[BARO_SPEED_NUM + 1];
	u16 baro_cnt[2];
	float baro_speed;
	float old_z_speed_by_kf;
	float forward_z_speed;
	float height_ctrl_out;
	float init_wz_acc;
	float wz_acc;
	float wz_acc_old[100];
	float z_speed;
	float z_height, z_height_old;

	float wz_speed, wz_speed_old;
	  char   ultra_error;
	float wz_acc_mms2;
	float wz_acc_mms2_lp;


	u8 baro_ctrl_start;
	float baro_height, baro_height_old;
	float baro_measure;


	float exp_height_speed, exp_height;
	float ultra_speed;
	float ultra_dis_lpf;
	float ultra_ctrl_out;
	float ultra_sp_tmp, ultra_dis_tmp;

	char height_reach_cnt;
	_st_height_pid_v ultra_ctrl;
	_st_height_pid ultra_pid;

	_st_height_pid_v wz_speed_pid_v;
	_st_height_pid wz_speed_pid;

	_st_height_pid_v acc_pid_v;
	_st_height_pid   acc_pid;

	  FLY_UP_DOWN fly_up_and_down;
	  float z_height_loop_hz1  ;
	  float speed_lp_hz2;
	  float z_height_ki  ;
	  float z_height_ki_2;
	  float acc_death;
	  float wz_death;
	  float height_ki;
	 
	  //外环
		void height_ctrl_outer(float T, float thr, float height,float speed);
		//内环环
		void height_speed_ctrl(float T, float thr, float exp_z_speed, float h_speed);
		//加速度换
		void height_acc_ctrl(float T, float thr, float exp_z_acc, float z_acc);


	  void Height_Ctrl(float T, float thr);

	  void Ultra_PID_Init(void);

	  void WZ_Speed_PID_Init(void);

	 

	  void Baro_Ctrl(float T, float thr);

	  void Ultra_Ctrl(float T, float thr);


	  void Ultra_speed_caculate(float T);
	  float self_land_ctrl(void);
	  float self_fly_up_ctrl(void);

	  /*---------与高度传感器融合得到高度、速度--------*/
	  void speed_height_caculate_with_sensor(float T );
	  void speed_height_caculate_with_baro(float T, float baro_height, float baro_speed);
	  void speed_height_caculate_with_baro_gps(float T, float baro_height, float baro_speed,
		  float gps_height);

private:

};
 
extern HLT_CTRL hlt_ctl;




#endif

