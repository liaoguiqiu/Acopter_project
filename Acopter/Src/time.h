#ifndef _TIME_H_
#define _TIME_H_
#include "include.h" 
 

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

//时间计数
enum
{
	NOW = 0,
	OLD,
	NEW,
};


class systime
{
public:
	/*---变量---*/
volatile uint32_t sysTickUptime;
int time_1h, time_1m, time_1s, time_1ms;

float Cycle_T[GET_TIME_NUM][3];

/*---方法---*/
void TIM_INIT(void);
void sys_time(void);

u16 Get_Time(u8, u16, u16);

float Get_Cycle_T(u8);

void Cycle_Time_Init(void);

uint32_t GetSysTime_us(void);

private:

};
 
extern systime time;



#endif