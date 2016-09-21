#include "pwm.h"
#include "rc.h"
//·­×ªÊ±¼ä
#define PWM_FULL_TIME 1800
#define MOTOR_STOP_TIME 1000


void SetPwm(int16_t pwm[], s16 min, s16 max)
{
	u8 i;
	s16 pwm_tem[MAXMOTORS];

	for (i = 0; i<MAXMOTORS; i++)
	{
		pwm_tem[i] = pwm[i];
		pwm_tem[i] = (short)LIMIT(pwm_tem[i], min, max);
		pwm_tem[i] = pwm_tem[i] * (PWM_FULL_TIME - MOTOR_STOP_TIME) / 1000 + MOTOR_STOP_TIME;

	}

	if (rc.fly_ready)
	{

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_tem[0]);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_tem[1]);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_tem[2]);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm_tem[3]);

	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_STOP_TIME);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_STOP_TIME);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_STOP_TIME);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_STOP_TIME);

	}



	//	
	//	TIM1->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[0]] ) + INIT_DUTY;				//1	
	//	TIM1->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[1]] ) + INIT_DUTY;				//2
	//	TIM1->CCR2 = PWM_RADIO *( pwm_tem[CH_out_Mapping[2]] ) + INIT_DUTY;				//3	
	//	TIM1->CCR1 = PWM_RADIO *( pwm_tem[CH_out_Mapping[3]] ) + INIT_DUTY;				//4
}
u16 Rc_Pwm_In[RC_CH_NUM];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static int capture[RC_CH_NUM][2];
	static uint8_t flag[RC_CH_NUM];

	rc.Feed_Rc_Dog(1);//RC
	if (htim->Instance == TIM3)
	{

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
			{
				capture[CH1][0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
				flag[CH1] = 0;
			}
			else
			{
				if (flag[CH1] == 0)
				{
					flag[CH1] = 1;
					capture[CH1][1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
					if (capture[CH1][1] > capture[CH1][0])
					{
						Rc_Pwm_In[CH1] = capture[CH1][1] - capture[CH1][0];
					}
					else
					{
						Rc_Pwm_In[CH1] = 0xFFFF - capture[CH1][0] + capture[CH1][1];
					}
				}
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)
			{
				capture[CH2][0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
				flag[CH2] = 0;
			}
			else
			{
				if (flag[CH2] == 0)
				{
					flag[CH2] = 1;
					capture[CH2][1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
					if (capture[CH2][1] > capture[CH2][0])
					{
						Rc_Pwm_In[CH2] = capture[CH2][1] - capture[CH2][0];
					}
					else
					{
						Rc_Pwm_In[CH2] = 0xFFFF - capture[CH2][0] + capture[CH2][1];
					}
				}
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)
			{
				capture[CH3][0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
				flag[CH3] = 0;
			}
			else
			{
				if (flag[CH3] == 0)
				{
					flag[CH3] = 1;
					capture[CH3][1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
					if (capture[CH3][1] > capture[CH3][0])
					{
						Rc_Pwm_In[CH3] = capture[CH3][1] - capture[CH3][0];
					}
					else
					{
						Rc_Pwm_In[CH3] = 0xFFFF - capture[CH3][0] + capture[CH3][1];
					}
				}
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)
			{
				capture[CH4][0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
				flag[CH4] = 0;
			}
			else
			{
				if (flag[CH4] == 0)
				{
					flag[CH4] = 1;
					capture[CH4][1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
					if (capture[CH4][1] > capture[CH4][0])
					{
						Rc_Pwm_In[CH4] = capture[CH4][1] - capture[CH4][0];
					}
					else
					{
						Rc_Pwm_In[CH4] = 0xFFFF - capture[CH4][0] + capture[CH4][1];
					}
				}
			}
		}

	}

	else   if (htim->Instance == TIM2)

	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
			{
				capture[CH5][0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
				flag[CH5] = 0;
			}
			else
			{
				if (flag[CH5] == 0)
				{
					flag[CH5] = 1;
					capture[CH5][1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
					if (capture[CH5][1] > capture[CH5][0])
					{
						Rc_Pwm_In[CH5] = capture[CH5][1] - capture[CH5][0];
					}
					else
					{
						Rc_Pwm_In[CH5] = 0xFFFF - capture[CH5][0] + capture[CH5][1];
					}
				}
			}
		}
		else   if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET)
			{
				capture[CH6][0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
				flag[CH6] = 0;
			}
			else
			{
				if (flag[CH6] == 0)
				{
					flag[CH6] = 1;
					capture[CH6][1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
					if (capture[CH6][1] > capture[CH6][0])
					{
						Rc_Pwm_In[CH6] = capture[CH6][1] - capture[CH6][0];
					}
					else
					{
						Rc_Pwm_In[CH6] = 0xFFFF - capture[CH6][0] + capture[CH6][1];
					}
				}
			}
		}


	}
	else if (htim->Instance == TIM12)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET)
			{
				capture[CH7][0] = HAL_TIM_ReadCapturedValue(&htim12, TIM_CHANNEL_1);
				flag[CH7] = 0;
			}
			else
			{
				if (flag[CH7] == 0)
				{
					flag[CH7] = 1;
					capture[CH7][1] = HAL_TIM_ReadCapturedValue(&htim12, TIM_CHANNEL_1);
					if (capture[CH7][1] > capture[CH7][0])
					{
						Rc_Pwm_In[CH7] = capture[CH7][1] - capture[CH7][0];
					}
					else
					{
						Rc_Pwm_In[CH7] = 0xFFFF - capture[CH7][0] + capture[CH7][1];
					}
				}
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET)
			{
				capture[CH8][0] = HAL_TIM_ReadCapturedValue(&htim12, TIM_CHANNEL_2);
				flag[CH8] = 0;
			}
			else
			{
				if (flag[CH8] == 0)
				{
					flag[CH8] = 1;
					capture[CH8][1] = HAL_TIM_ReadCapturedValue(&htim12, TIM_CHANNEL_2);
					if (capture[CH8][1] > capture[CH8][0])
					{
						Rc_Pwm_In[CH8] = capture[CH8][1] - capture[CH8][0];
					}
					else
					{
						Rc_Pwm_In[CH8] = 0xFFFF - capture[CH8][0] + capture[CH8][1];
					}
				}
			}
		}


	}



}
