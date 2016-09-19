#include "parameter.h"
#include  "flash.h"
#include  "delay.h"
#include "AHRS.h"
#include "magnet.h"
#include "ctrl.h"
#include "sonar.h"
#include "height_ctrl.h"

PARAMERTER parameter;


void PARAMERTER:: Para_Init()
{


	if (  flash_save.flash_read_parameters()  == 1)
	{

		 
		 
		delay_ms(500);
		//                   SHOT_MAX_PWM=1300;
		//                SHOT_MIN_PWM=1050;
		//                PLANT_FORM=1650;
		//                 MAX_PLANT_FORM=2200;

	}
	else
	{
		 ahrs.Gyro_CALIBRATE = 0;
		 ahrs.Gyro_Offset.x = -24;
		 ahrs.Gyro_Offset.y = -25;
		 ahrs.Gyro_Offset.z = -1;

		 ahrs.Acc_CALIBRATE = 0;

		 ahrs.Acc_Offset.x = -290;
		 ahrs.Acc_Offset.y = 47;
		 ahrs.Acc_Offset.z = 57; //-9865 
		 mag_s. Mag_CALIBRATED = 0;
		mag_s.Mag_Offset.x = 186;
		mag_s.Mag_Offset.y = 283;
		mag_s.Mag_Offset.z = 181;


		 
		/*--Ͷ�����--*/
		PID_Para_Init();
	}
}
void PARAMERTER:: PID_Para_Init()
{
	/* PID Ĭ��ֵ */
	ctrl_s.ctrl_1.PID[PIDROLL].kp = 0.7;
	ctrl_s.ctrl_1.PID[PIDPITCH].kp = 0.7;
	ctrl_s.ctrl_1.PID[PIDYAW].kp = 1.5;
	 

	ctrl_s.ctrl_1.PID[PIDROLL].ki = 0.09;
	ctrl_s.ctrl_1.PID[PIDPITCH].ki = 0.09;
	ctrl_s.ctrl_1.PID[PIDYAW].ki = 1.5;
	 

	ctrl_s.ctrl_1.PID[PIDROLL].kd = 1.6;
	ctrl_s.ctrl_1.PID[PIDPITCH].kd = 1.6;
	ctrl_s.ctrl_1.PID[PIDYAW].kd = 1.6;

 
	ctrl_s.ctrl_1.PID[PIDROLL].kdamp = 0;
	ctrl_s.ctrl_1.PID[PIDPITCH].kdamp= 0;
	ctrl_s.ctrl_1.PID[PIDYAW].kdamp = 0;

	ctrl_s.ctrl_2.PID[PIDROLL].kp = 0.75;
	ctrl_s.ctrl_2.PID[PIDPITCH].kp = 0.75;
	ctrl_s.ctrl_2.PID[PIDYAW].kp = 0.57;


	ctrl_s.ctrl_2.PID[PIDROLL].ki= 0.022;
	ctrl_s.ctrl_2.PID[PIDPITCH].ki = 0.022;
	ctrl_s.ctrl_2.PID[PIDYAW].ki = 0.020;
	 


	ctrl_s.ctrl_2.PID[PIDROLL].kd = 0.460;
	ctrl_s.ctrl_2.PID[PIDPITCH].kd = 0.460;
	ctrl_s.ctrl_2.PID[PIDYAW].kd = 0.30;

	ctrl_s.Ctrl_Para_Init();
	hlt_ctl. WZ_Speed_PID_Init();
	hlt_ctl.Ultra_PID_Init();

}