#include "scheduler.h"
#include "sys.h"
#include "delay.h"
#include "stm32f4xx_it.h"
#include "includes.h"
#include "time.h"
#include "datatrans.h"
#include "rc.h"
#include "pwm.h"
#include "AHRS.h"
#include "magnet.h"
#include "imu.h"
#include "flash.h"
#include "parameter.h"
#include "ctrl.h"
#include  "led.h"
#include "height_ctrl.h"
#include "baro.h"
#include "gps.h"
#include "position.h"
#include "pos_ctrl.h"
#include "gps_kalman_v3.h"
#include "gps_kalman_v4.h"
#include "imu_ekf2.h"
#include "LQR_ctrl.hpp"
Scheduler scheduler;

/////////////////////////UCOSII任务设置///////////////////////////////////

//精准周期任务(周期绝对正确，且运行时间短的任务)
//设置任务优先级
#define  CYCLEST_TASK_PRIO       			1 
//设置任务堆栈大小
#define CYCLEST_STK_SIZE  		    		8048
//任务堆栈	
OS_STK CYCLEST_TASK_STK[CYCLEST_STK_SIZE];
//任务函数
OS_EVENT * sem_cyclest;		// cyvlest号量指针

void cyclest_task(void *pdata)
{
	u8 err;
	while (1)
	{
		OSSemPend(sem_cyclest, 0, &err);
		{
			scheduler.cyclest_loop();

			scheduler.loop.check_flag_cyclest = 0;		//循环运行完毕标志
		}
	}
}


//周期准确，运行时间较长，可分时间片段运行任务
//设置任务优先级
#define CYCLER_TASK_PRIO       			2
//设置任务堆栈大小
#define CYCLER_STK_SIZE  				8024
//任务堆栈
OS_STK CYCLER_TASK_STK[CYCLER_STK_SIZE];
//任务函数

OS_EVENT * sem_cycler;		// 信号量指针
 
void cycler_task(void *pdata)
{

	u8 err1;
	while (1)
	{
		OSSemPend(sem_cycler, 0, &err1);
		{
		
			scheduler.cycler_loop();
			scheduler.loop.check_flag_cycler=0;		//循环运行完毕标志
		}
 
	}
}


//次优先级任务
//设置任务优先级
#define OUTER_TASK_PRIO       			3
//设置任务堆栈大小
#define OUTER_STK_SIZE 				1024
//任务堆栈
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
//任务函数
void  outer_task(void *pdata)
{
	while (1)
	{
            scheduler.outer_loop();
	}
}


//START 任务
//设置任务优先级
//#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
////设置任务堆栈大小
//#define START_STK_SIZE  				64
////任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr = 0;
	pdata = pdata;
	sem_cyclest = OSSemCreate(0);		//创建信号量
	sem_cycler = OSSemCreate(0);		//创建信号量


	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
	OSTaskCreate(cyclest_task, (void *)0, (OS_STK*)&CYCLEST_TASK_STK[CYCLEST_STK_SIZE - 1], CYCLEST_TASK_PRIO);
	OSTaskCreate(cycler_task, (void *)0, (OS_STK*)&CYCLER_TASK_STK[CYCLER_STK_SIZE - 1], CYCLER_TASK_PRIO);
	OSTaskCreate(outer_task, (void *)0, (OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE - 1], OUTER_TASK_PRIO);

	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();

}

void Scheduler:: Loop_check()  //TIME INTTERRUPT
  {
          loop.time++; //u16
          loop.cnt_1ms++;
          loop.cnt_3ms++;
          loop.cnt_5ms++;
          loop.cnt_10ms++;
          loop.cnt_20ms++;
          loop.cnt_50ms++;
		  loop.cnt_100ms++;
          loop2.cnt_1ms++;
          loop2.cnt_3ms++;
          loop2.cnt_5ms++;
          loop2.cnt_10ms++;
          loop2.cnt_20ms++;
          loop2.cnt_50ms++;
		  loop2.cnt_100ms++;
          // loop.cnt_100ms++;
        if( loop.check_flag_cyclest == 1)
        {
        loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
        }
	    else
        {	
          loop.check_flag_cyclest = 1;	//该标志位在循环的最后被清零
          OSSemPost(sem_cyclest);
          
         
          
           if( loop.check_flag_cycler==0)
          {
			  loop.check_flag_cycler= 1;	//该标志位在循环的最后被清零
			  OSSemPost(sem_cycler);

          }
         
        }
       
        
        
}

//严格周期任务
void Scheduler::cyclest_loop()
{
        if(loop.cnt_1ms>=1)
        {
          loop.cnt_1ms=0;
          cyclest_Duty_1ms();
          
        }
	if (loop.cnt_3ms >= 3)
	{


		loop.cnt_3ms = 0;
		cyclest_Duty_3ms();			//周期5ms的任务
	}

	if (loop.cnt_5ms >= 5)
	{


		loop.cnt_5ms = 0;
		cyclest_Duty_5ms();			//周期5ms的任务
	}
        if (loop.cnt_10ms >= 10)
	{


		loop.cnt_10ms = 0;
                cyclest_Duty_10ms();			//周期10ms的任务
		 
	}
        if (loop.cnt_20ms >= 20)
	{
          
          
                loop.cnt_20ms = 0;
                cyclest_Duty_20ms();			//周期5ms的任务
	}
        if (loop.cnt_50ms >= 50)
	{
          
          
          loop.cnt_50ms = 0;
          cyclest_Duty_50ms();			//周期5ms的任务
	}
		if (loop.cnt_100ms>=100)
		{
			loop.cnt_100ms = 0;
			cyclest_Duty_100ms();			//周期5ms的任务

		}
}


void Scheduler::cycler_loop()
{
        if(loop2.cnt_1ms>=1)
        {
          loop2.cnt_1ms=0;
          cycler_Duty_1ms();
          
        }
	if (loop2.cnt_3ms >= 3)
	{


		loop2.cnt_3ms = 0;
		cycler_Duty_3ms();			//周期5ms的任务
	}

	if (loop2.cnt_5ms >= 5)
	{
	
		loop2.cnt_5ms = 0;
		cycler_Duty_5ms();			//周期5ms的任务
	}
        if (loop2.cnt_10ms >= 10)
	{


		loop2.cnt_10ms = 0;
                cycler_Duty_10ms();			//周期10ms的任务
		 
	}
        if (loop2.cnt_20ms >= 20)
	{
          
          
                loop2.cnt_20ms = 0;
                cycler_Duty_20ms();			//周期5ms的任务
	}
        if (loop2.cnt_50ms >= 50)
	{
          
          
          loop2.cnt_50ms = 50;
          cycler_Duty_50ms();			//周期5ms的任务
	}
	

}

void Scheduler::outer_loop()
{

	 

}


//cycler
void Scheduler::cycler_Duty_1ms()
{

	 
      
}



void Scheduler::cycler_Duty_3ms()
{

	 

}

void Scheduler::cycler_Duty_5ms()
{
	
}

void Scheduler::cycler_Duty_10ms()
{        
	//上下位机通信
#if (CONFIG_SET_UP_GPS_ON==0)
	data_trans.data_trans_with_ano();
#endif
	

	float data_time8 = time.Get_Cycle_T(8);
	 

     if (time.time_1s>22 && imu_ekf2.Filter_ON == 0)
	{
		imu_ekf2.Filter_ON = 1;
	}
	 if (time.time_1s>30)
	{
		imu_ekf2.Filter_ON = 2; 
	}
	if (imu_ekf2.Filter_ON)
	{
		//ttttt1 = time.GetSysTime_us() / 1000000.0f; //本次的时间   
		imu_ekf2.Imu_emistate_altitude_ekf(data_time8, ahrs.Gyro_deg.x, ahrs.Gyro_deg.y, ahrs.Gyro_deg.z,           //789
			ahrs.Acc.x, ahrs.Acc.y, ahrs.Acc.z,         //10 11 12
			//  (ahrs.Acc.x - position.line_acc_err(0, 0)), (ahrs.Acc.y - position.line_acc_err(1, 0)), (ahrs.Acc.z - position.line_acc_err(2, 0)),         //10 11 12
			mag_s.Mag_Val.x, mag_s.Mag_Val.y, mag_s.Mag_Val.z,
			//imu_dcm.mag_sim_3d.x, imu_dcm.mag_sim_3d.y, imu_dcm.mag_sim_3d.z,
			gps.px, gps.py, gps.pz,
			gps.vn, gps.ve, gps.vd);
	}
	else
	{
		imu_ekf2.IMU_EKF2_init_ststes(imu_dcm.Roll, imu_dcm.Pitch, imu_dcm.Yaw, 0, 0, 0);
		//imu_ekf2.IMU_EKF2_init_ststes(0, 0, imu_dcm.Yaw, 0, 0, 0);
	}
	gps_po_kf_v4.speed_and_acc_debias(data_time8, -imu_ekf2.V_b_forward(0, 0)*1000, imu_ekf2.V_b_forward(1, 0)*1000,
	               	gps.gps_v_x_b * 1000, gps.gps_v_y_b * 1000,
					- imu_ekf2.line_acc_b(0, 0) * 1000, imu_ekf2.line_acc_b(1, 0) * 1000 );
}


void Scheduler::cycler_Duty_20ms()
{
           
        
}

void Scheduler::cycler_Duty_50ms()
{
  
}

//严格周期
void Scheduler::cyclest_Duty_1ms()
{
  float data_time1 = time.Get_Cycle_T(0);
  /*-------DMA完成后对读取数据进行处理------*/
  if (ahrs. mpu_data_ok == 1)
  {
          ahrs.mpu_data_ok = 0;
           ahrs.ahrs_Data_Prepare (data_time1);			//mpu6轴传感器数据处理
           mag_s.Read_Mag_Data();
  }
  /*-------在这里打开DMA------*/
  ahrs.MPU_READ_INITIAL_DATA();
  gps.GPS_pre(data_time1);
  
}



void Scheduler::cyclest_Duty_3ms()
{

	float data_time3 = time.Get_Cycle_T(1);

	ahrs.ahrs_data_filt(data_time3);

	mag_s.filt_data();

	rc.RC_Duty(data_time3, Rc_Pwm_In);
	
	hlt_ctl.speed_height_caculate_with_sensor(data_time3 );
	hlt_ctl. height_speed_ctrl(data_time3, ctrl_s.thr,hlt_ctl. ultra_ctrl_out,hlt_ctl. forward_z_speed);
	//position.pos_speed_displacement_with_gps(data_time3);


	ctrl_s.CTRL_1(data_time3);
	 

        
}

void Scheduler::cyclest_Duty_5ms()
{

  float data_time5 = time.Get_Cycle_T(2);
  
  imu_dcm.IMUupdate(0.5f *data_time5, ahrs.Gyro_deg.x, ahrs.Gyro_deg.y, ahrs.Gyro_deg.z, 
	  ahrs.Acc.x, ahrs.Acc.y, ahrs.Acc.z,
	  &imu_dcm.Roll, &imu_dcm.Pitch, &imu_dcm.Yaw);

  // 
  // 		tttttt2 = time.GetSysTime_us() / 1000000.0f; //本次的时间
  //tttttt3 = tttttt2 - ttttt1;


  ctrl_s.CTRL_2(data_time5);
        
}

void Scheduler::cyclest_Duty_10ms()
{
	float data_time_10ms = time.Get_Cycle_T(9);

	//速度控制仿真
    #if SIMULINK_ON
	simulink.simu_speed_ctrl(data_time_10ms);

    #else

	/*********************速度控制开始*********************************************/
	if (pos_ctrl.Postion_hold_ctrl_on)
	{
		pos_ctrl.speed_ctrl(data_time_10ms, ctrl_s.Thr_Weight,
			/* - imu_ekf2.xhat_k(4,0)*1000,
			imu_ekf2.xhat_k(5, 0) * 1000,*/
			/*	-imu_ekf2.line_acc_b(0, 0)*1000, imu_ekf2.line_acc_b(1, 0)*1000,
			-imu_ekf2.V_b_forward(0, 0) * 1000,	imu_ekf2.V_b_forward(1, 0) * 1000*/
			gps_po_kf_v4.acc_b_debias.x, gps_po_kf_v4.acc_b_debias.y,
			gps_po_kf_v4.speed_b_debias.x, gps_po_kf_v4.speed_b_debias.y
			//position.xkf_vn_b, position.xkf_ve_b,
			//   gps.gps_v_x_b * 1000, gps.gps_v_y_b * 1000,
			// position.x_pos_hf, position.y_pos_hf);
			//  gps.gps_pos_x_b * 1000, gps.gps_pos_y_b * 1000

			);

	}
	else
	{
		pos_ctrl.para_clear();
	}
    #endif
	

      if (baro. MS5611_Update()) 				//¸üÐÂms5611ÆøÑ¹¼ÆÊý¾Ý
      {
		  float data_time = time.Get_Cycle_T(6);
		  //高度外环控制
		hlt_ctl.  height_ctrl_outer(data_time, ctrl_s. thr, baro.high_kf_pos_acc, hlt_ctl.forward_z_speed);

	   //20ms
      }

       
     
    
}


void Scheduler::cyclest_Duty_20ms()
{
  //
  float data_time=time.Get_Cycle_T(7);

  gps_po_kf_v4.gps_pos_v4_fuse(data_time, gps.gps_pos_x_b * 1000, gps.gps_pos_y_b * 1000,
	  -imu_ekf2.V_b_lag(0, 0) * 1000,
	  imu_ekf2.V_b_lag(1, 0) * 1000);
 // gps_kalman_v3_update.Gps_kalman_v3_xy_update(data_time);

  //position.caculate_line_acc_err();


}

void Scheduler::cyclest_Duty_50ms()
{
	float data_time6 = time.Get_Cycle_T(4);
   
	check_flash_save();

	led.led_change();
         
}

void Scheduler::cyclest_Duty_100ms()
{

	/*********************定点开始*********************************************/
#if SIMULINK_ON
	simulink.simu_pos_ctrl(0.1);
#else

	if (pos_ctrl.Postion_hold_ctrl_on)
	{
		pos_ctrl.position_ctrl(0.1, ctrl_s.Thr_Weight,
			/* - imu_ekf2.xhat_k(4,0)*1000,
			imu_ekf2.xhat_k(5, 0) * 1000,*/
			-imu_ekf2.V_b_forward(0, 0) * 1000,
			imu_ekf2.V_b_forward(1, 0) * 1000,
			//position.xkf_vn_b, position.xkf_ve_b,
			//   gps.gps_v_x_b * 1000, gps.gps_v_y_b * 1000,
			// position.x_pos_hf, position.y_pos_hf);
			gps.gps_pos_x_b * 1000, gps.gps_pos_y_b * 1000
			//gps_po_kf_v4.x_b, gps_po_kf_v4.y_b
			);

	}
	else
	{
		pos_ctrl.para_clear();
	}
	/************************定点结束************************************************/

#endif
	
}

void Scheduler:: check_flash_save()
{
	if (flash_save.save_on == 1)
	{
		flash_save.save_on = 0;
		flash_save.flash_save_parameters(); 
		flash_save.save_time = 100;
	}
	if (flash_save.save_time)
	{
		flash_save.save_time--;
	}
}
