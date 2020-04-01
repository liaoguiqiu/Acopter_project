#include "time.h"  
#include "scheduler.h"


systime time;
 void systime:: sys_time()
{ 
	if( !Init_Finish )
	{
		return;
	}
        
        
	scheduler.Loop_check();
        
        
        
  if(time_1ms < 999)
	{
    time_1ms++;

		
	}
	else
	{
		
    time_1ms =0;
	  if(time_1s<59)
	  {
      time_1s++;
		}
		else
		{
			time_1s = 0;
			if(time_1m<59)
			{
				time_1m++;
			}
			else
			{
				time_1m = 0;
				if(time_1h<23)
				{
					time_1h++;
				}
				else
				{
					time_1h = 0;
				}
			}
		}
	}
}

float systime::Get_Cycle_T(u8 item)
{
    
  
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
	
     
        return Cycle_T[item][NEW];
}

void systime::Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}

}

uint32_t systime::GetSysTime_us(void)
{
    
//  static int s_load;
//   static int s_val;
//  s_load=SysTick->LOAD ;
//  s_val=SysTick->VAL; 
  
	register uint32_t ms;
        uint32_t s_load;
         uint32_t s_val;
	u32 value;
        s_load =SysTick->LOAD ;
        s_val=SysTick->VAL;
        
	ms = sysTickUptime;
         value = ms * TICK_US + (s_load- s_val) * TICK_US / s_load;
	//value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}
  