#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_


#include "include.h" 
#include "sys.h"
#include "delay.h"
#include "includes.h"

//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64

extern OS_STK START_TASK_STK[START_STK_SIZE];
//任务堆栈	
void  cyclest_task(void *pdata);
void  cycler_task(void *pdata);
void  outer_task(void *pdata);
void   start_task(void *pdata);

typedef struct
{
	u8 check_flag_cyclest;
	u8 check_flag_cycler;
	u8 err_flag;
	s16 cnt_1ms;
	s16 cnt_3ms;
	s16 cnt_5ms;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	s16 cnt_100ms;
	// s16 cnt_100ms;
	u16 time;
}loop_t;


class Scheduler
{
public:
     float ttttt1,tttttt2,tttttt3;
	s16 loop_cnt;
	loop_t loop;
	loop_t loop2;
	 
/*------方法--------*/

	void Loop_check(void);
        
	void cyclest_loop();
	void cycler_loop();
	void outer_loop();
        
	void cyclest_Duty_1ms();
	void cyclest_Duty_3ms();
	void cyclest_Duty_5ms();
	void cyclest_Duty_10ms();
	void cyclest_Duty_20ms();
	void cyclest_Duty_50ms();
	void cyclest_Duty_100ms();
        void cycler_Duty_1ms();
	void cycler_Duty_3ms();
	void cycler_Duty_5ms();
	void cycler_Duty_10ms();
	void cycler_Duty_20ms();
	void cycler_Duty_50ms();
        
	void check_flash_save();
private:

};
 

extern Scheduler scheduler;


  
  
#endif

