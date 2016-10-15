#ifndef __INCLUDE_H
#define __INCLUDE_H

#include "stm32f4xx_hal.h"
#include "sys.h"
#include "vector3.h"


#define MAXMOTORS 		(4)		//电机数量
#define GET_TIME_NUM 	(10)		//设置获取时间的数组数量
#define CH_NUM 				(8) 	//接收机通道数量
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度
#define TO_ANGLE 				0.06103f 		//0.061036 //   4000/65536  +-2000   ???


typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;
typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_Y ,
 G_X ,
 G_Z ,
 TEM ,
 ITEMS ,
};
// CH_filter[],0横滚，1俯仰，2油门，3航向		
enum
{
 PIT= 0,
 ROL ,
 THR ,
 YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
};
//================控制=====================
#define MAX_CTRL_ANGLE			25.0f										//遥控能达到的最大角度
#define ANGLE_TO_MAX_AS 		30.0f										//角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE		//外环积分幅度
#define MAX_CTRL_ASPEED 	 	300.0f									//ROL,PIT允许的最大控制角速度
#define MAX_CTRL_YAW_SPEED 	150.0f									//YAW允许的最大控制角速度
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED		//内环积分幅度
#define MAX_PWM				100			///%	最大PWM输出为100%油门
#define MAX_THR       60 			///%	油门通道最大占比80%，留20%给控制量
#define READY_SPEED   10			///%	解锁后电机转速20%油门
//=========================================


extern char Init_Finish;
extern UART_HandleTypeDef huart2;  
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim12;
extern SPI_HandleTypeDef hspi3;

extern SPI_HandleTypeDef hspi1;
#endif