#ifndef _CTRL_H
#define	_CTRL_H
 
#include "include.h"
#include "rc.h"
#include "imu.h"
#include "parameter.h"
 
enum PIDItem
{
	PIDROLL = 0,
	PIDPITCH,
	PIDYAW,
	PID4,
	PID5,
	PID6,
	PIDITEMS
};



typedef struct
{
	Vector3f err;
	Vector3f err_old;
	Vector3f err_i;
	Vector3f eliminate_I;
	Vector3f err_d;
	Vector3f damp;
	Vector3f out;
	pid_t 	PID[PIDITEMS];
	Vector3f err_weight;
	float FB;

}ctrl_t;


class CTRL_S
{
public:
	 
	ctrl_t ctrl_1;//内环
	ctrl_t ctrl_2;//外环

	Vector3f except_A ;

	//Vector3f ctrl_angle_offset = {-1.15 ,-1.3 ,0};
	Vector3f ctrl_angle_offset ;//{-0.2,0.9,0};
	Vector3f compensation;

	Vector3f except_AS;




	float thr_value;
	u8 Thr_Low;
	float Thr_Weight;

	float thr;
	float g_old[ITEMS];


	float motor[MAXMOTORS];
	float posture_value[MAXMOTORS];
	float curve[MAXMOTORS];
	s16 last_motor_out[4];
	CTRL_S()
	{

		except_A.x = 0;
		except_A.y = 0;
		except_A.z = 0;
		 
		
		ctrl_angle_offset.x = 0;//{-0.2,0.9,0};
		ctrl_angle_offset.y = 0;//{-0.2,0.9,0};
		ctrl_angle_offset.z = 0;//{-0.2,0.9,0};

		ctrl_1.PID[PIDROLL].kdamp = 1;
		ctrl_1.PID[PIDPITCH].kdamp = 1;
		ctrl_1.PID[PIDYAW].kdamp = 1;

		ctrl_1.FB = 0.1;   //外  0<fb<1
	}



	void CTRL_2(float);
	void CTRL_1(float);
	void Ctrl_Para_Init(void);
	void Thr_Ctrl(float);
	void All_Out(float out_roll, float out_pitch, float out_yaw);

private:

};
 

extern CTRL_S ctrl_s;
#endif

