#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "include.h"
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
	float err_i;
	float pid_out;

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
	short ULTRA_MAX_HEIGHT; // mm  
	float baro_speed_arr[BARO_SPEED_NUM + 1];
	u16 baro_cnt[2];
	float baro_speed;

	float height_ctrl_out;
	float wz_acc;
	float z_speed;
	float z_height, z_height_old;

	float wz_speed, wz_speed_old;
	  char   ultra_error;
	float wz_acc_mms2;


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
	  FLY_UP_DOWN fly_up_and_down;
	
	    HLT_CTRL( )
	  {
		  ULTRA_MAX_HEIGHT = 500;
	  
	  }


		void height_ctrl_outer(float T, float thr, float height,float speed);
	
	  void Height_Ctrl(float T, float thr);

	  void Ultra_PID_Init(void);

	  void WZ_Speed_PID_Init(void);

	  void height_speed_ctrl(float T, float thr, float exp_z_speed, float);

	  void Baro_Ctrl(float T, float thr);

	  void Ultra_Ctrl(float T, float thr);


	  void Ultra_speed_caculate(float T);
	  float self_land_ctrl(void);
	  float self_fly_up_ctrl(void);

	  /*---------与高度传感器融合得到高度、速度--------*/
	  void speed_height_caculate_with_sensor(float T, float sensor_height, float sensor_speed);

private:

};
 
extern HLT_CTRL hlt_ctl;




#endif

