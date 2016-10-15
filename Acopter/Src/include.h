#ifndef __INCLUDE_H
#define __INCLUDE_H

#include "stm32f4xx_hal.h"
#include "sys.h"
#include "vector3.h"


#define MAXMOTORS 		(4)		//�������
#define GET_TIME_NUM 	(10)		//���û�ȡʱ�����������
#define CH_NUM 				(8) 	//���ջ�ͨ������
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	�Ƕ�ת����
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
// CH_filter[],0�����1������2���ţ�3����		
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
//================����=====================
#define MAX_CTRL_ANGLE			25.0f										//ң���ܴﵽ�����Ƕ�
#define ANGLE_TO_MAX_AS 		30.0f										//�Ƕ����Nʱ���������ٶȴﵽ��󣨿���ͨ������CTRL_2��Pֵ������
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE		//�⻷���ַ���
#define MAX_CTRL_ASPEED 	 	300.0f									//ROL,PIT����������ƽ��ٶ�
#define MAX_CTRL_YAW_SPEED 	150.0f									//YAW����������ƽ��ٶ�
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED		//�ڻ����ַ���
#define MAX_PWM				100			///%	���PWM���Ϊ100%����
#define MAX_THR       60 			///%	����ͨ�����ռ��80%����20%��������
#define READY_SPEED   10			///%	��������ת��20%����
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