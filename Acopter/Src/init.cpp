#include "init.h"

#include "sys.h"
#include "delay.h"
#include "includes.h"
#include "time.h"
#include "scheduler.h"
#include "AHRS.h"
#include "baro.h"

char Init_Finish=0;


void All_Init()
{
        delay_init(168);		  //初始化延时函数




        
        delay_ms(10 ); 
        /*-----参数---*/
        parameter.Para_Init();
        
        /*-----气压计-------*/
		baro.MS5611_Init();
	//	baro.MS5611_Init();
        /*---陀螺仪----*/
        ahrs. MPU9250_Init(&hspi3);
        while (ahrs. MPU9250_Check() != SUCCESS)
          asm("nop");
        delay_ms(10);
        
        
     /*电机*/
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);    
          /*遥控器*/
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
    HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_2);
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
      

	time.Cycle_Time_Init();
	OSInit();
	OSTaskCreate(start_task, (void *)0, (OS_STK *)&START_TASK_STK[START_STK_SIZE - 1], START_TASK_PRIO);//创建起始任务
	Init_Finish = 1;
	OSStart();
}