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


Scheduler scheduler;

/////////////////////////UCOSII��������///////////////////////////////////


//��׼��������(���ھ�����ȷ��������ʱ��̵�����)
//�����������ȼ�
#define  CYCLEST_TASK_PRIO       			1 
//���������ջ��С
#define CYCLEST_STK_SIZE  		    		2048
//�����ջ	
OS_STK CYCLEST_TASK_STK[CYCLEST_STK_SIZE];
//������
OS_EVENT * sem_cyclest;		// cyvlest����ָ��

void cyclest_task(void *pdata)
{
	u8 err;
	while (1)
	{
		OSSemPend(sem_cyclest, 0, &err);
		{
			scheduler.cyclest_loop();

			scheduler.loop.check_flag_cyclest = 0;		//ѭ��������ϱ�־
		}
	}
}


//����׼ȷ������ʱ��ϳ����ɷ�ʱ��Ƭ����������
//�����������ȼ�
#define CYCLER_TASK_PRIO       			2
//���������ջ��С
#define CYCLER_STK_SIZE  				1024
//�����ջ
OS_STK CYCLER_TASK_STK[CYCLER_STK_SIZE];
//������

OS_EVENT * sem_cycler;		// �ź���ָ��
 
void cycler_task(void *pdata)
{

	u8 err1;
	while (1)
	{
		OSSemPend(sem_cycler, 0, &err1);
		{
		
			scheduler.cycler_loop();
			scheduler.loop.check_flag_cycler=0;		//ѭ��������ϱ�־
		}
 
	}
}


//�����ȼ�����
//�����������ȼ�
#define OUTER_TASK_PRIO       			3
//���������ջ��С
#define OUTER_STK_SIZE 				1024
//�����ջ
OS_STK OUTER_TASK_STK[OUTER_STK_SIZE];
//������
void  outer_task(void *pdata)
{
	while (1)
	{

		scheduler.outer_loop();
	}
}


//START ����
//�����������ȼ�
//#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
////���������ջ��С
//#define START_STK_SIZE  				64
////�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr = 0;
	pdata = pdata;
	sem_cyclest = OSSemCreate(0);		//�����ź���
	sem_cycler = OSSemCreate(0);		//�����ź���


	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    
	OSTaskCreate(cyclest_task, (void *)0, (OS_STK*)&CYCLEST_TASK_STK[CYCLEST_STK_SIZE - 1], CYCLEST_TASK_PRIO);
	OSTaskCreate(cycler_task, (void *)0, (OS_STK*)&CYCLER_TASK_STK[CYCLER_STK_SIZE - 1], CYCLER_TASK_PRIO);
	OSTaskCreate(outer_task, (void *)0, (OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE - 1], OUTER_TASK_PRIO);

	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
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
          loop2.cnt_1ms++;
          loop2.cnt_3ms++;
          loop2.cnt_5ms++;
          loop2.cnt_10ms++;
          loop2.cnt_20ms++;
          loop2.cnt_50ms++;
          // loop.cnt_100ms++;
        if( loop.check_flag_cyclest == 1)
        {
        loop.err_flag ++;     //ÿ�ۼ�һ�Σ�֤��������Ԥ��������û�����ꡣ
        }
	    else
        {	
          loop.check_flag_cyclest = 1;	//�ñ�־λ��ѭ�����������
          OSSemPost(sem_cyclest);
          
         
          
           if( loop.check_flag_cycler==0)
          {
			  loop.check_flag_cycler= 1;	//�ñ�־λ��ѭ�����������
			  OSSemPost(sem_cycler);

          }
         
        }
       
        
        
}

//�ϸ���������
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
		cyclest_Duty_3ms();			//����5ms������
	}

	if (loop.cnt_5ms >= 5)
	{


		loop.cnt_5ms = 0;
		cyclest_Duty_5ms();			//����5ms������
	}
        if (loop.cnt_10ms >= 10)
	{


		loop.cnt_10ms = 0;
                cyclest_Duty_10ms();			//����10ms������
		 
	}
        if (loop.cnt_20ms >= 20)
	{
          
          
                loop.cnt_20ms = 0;
                cyclest_Duty_20ms();			//����5ms������
	}
        if (loop.cnt_50ms >= 50)
	{
          
          
          loop.cnt_50ms = 50;
          cyclest_Duty_50ms();			//����5ms������
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
		cycler_Duty_3ms();			//����5ms������
	}

	if (loop2.cnt_5ms >= 5)
	{


		loop2.cnt_5ms = 0;
		cycler_Duty_5ms();			//����5ms������
	}
        if (loop2.cnt_10ms >= 10)
	{


		loop2.cnt_10ms = 0;
                cycler_Duty_10ms();			//����10ms������
		 
	}
        if (loop2.cnt_20ms >= 20)
	{
          
          
                loop2.cnt_20ms = 0;
                cycler_Duty_20ms();			//����5ms������
	}
        if (loop2.cnt_50ms >= 50)
	{
          
          
          loop2.cnt_50ms = 50;
          cycler_Duty_50ms();			//����5ms������
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
           
        
}


void Scheduler::cycler_Duty_20ms()
{
           
        
}

void Scheduler::cycler_Duty_50ms()
{
           
        
}

//�ϸ�����
void Scheduler::cyclest_Duty_1ms()
{

	float data_time1 = time.Get_Cycle_T(0);
	/*-------DMA��ɺ�Զ�ȡ���ݽ��д���------*/
	if (ahrs. mpu_data_ok == 1)
	{
		ahrs.mpu_data_ok = 0;
		 ahrs.ahrs_Data_Prepare (data_time1);			//mpu6�ᴫ�������ݴ���
		 mag_s.Read_Mag_Data();
	}
	/*-------�������DMA------*/
	ahrs.MPU_READ_INITIAL_DATA();
	 



	  data_trans.data_trans_with_ano();
	
}



void Scheduler::cyclest_Duty_3ms()
{

	float data_time3 = time.Get_Cycle_T(1);

	ahrs.ahrs_data_filt(data_time3);

	mag_s.filt_data();

	rc.RC_Duty(data_time3, Rc_Pwm_In);

        
}

void Scheduler::cyclest_Duty_5ms()
{

  float data_time5 = time.Get_Cycle_T(2);
  
  imu_dcm.IMUupdate(0.5f *data_time5, ahrs.Gyro_deg.x, ahrs.Gyro_deg.y, ahrs.Gyro_deg.z, 
	  ahrs.Acc.x, ahrs.Acc.y, ahrs.Acc.z,
	  &imu_dcm.Roll, &imu_dcm.Pitch, &imu_dcm.Yaw);


        
}

void Scheduler::cyclest_Duty_10ms()
{
           
        
}


void Scheduler::cyclest_Duty_20ms()
{
           
        
}

void Scheduler::cyclest_Duty_50ms()
{
	
	check_flash_save();
}

void Scheduler:: check_flash_save()
{
	if (flash_save.save_on == 1)
	{
		flash_save.save_on = 0;
		flash_save.flash_save_parameters(); 
	}

}
