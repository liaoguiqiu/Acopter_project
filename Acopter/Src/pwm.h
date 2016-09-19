#ifndef __PWM_H
#define __PWM_H 
#include "stm32f4xx_hal.h"
#include "include.h"
#include "mymath.h"

enum inputpwm
{
	CH1 = 0,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7,
	CH8,
	RC_CH_NUM
};

extern u16 Rc_Pwm_In[RC_CH_NUM];
void SetPwm(int16_t pwm[], s16 min, s16 max);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif
