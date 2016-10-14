#include "led.h"
#include "rc.h"
#include "AHRS.h"
#include "magnet.h"
#include "flash.h"
LED led;

void LED ::led_change ()
{
	if (rc.fly_ready == 1)
	{
		if (rc.height_ctrl_mode==0)
		led_state0();

		else if (rc.height_ctrl_mode==1)
		led_state2();

	}
	else
	{
	
		 

		if (mag_s.Mag_CALIBRATED >= 1 
			||ahrs.Acc_CALIBRATE==1
			||ahrs.Gyro_CALIBRATE==1
			||flash_save.save_time>=1)
		{
			if (mag_s.Mag_CALIBRATED==1)
			led_state0();
			else if (mag_s.Mag_CALIBRATED == 2)
				led_state2();
			else
				led_state0();

		}
		else
		{
			led_state1();
		}


	}





}
void LED::led_state0()
{

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);


}


void LED::led_state1()
{

	 

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);


}


void LED::led_state2()
{
	if (on_time)
	{
		on_time--;
		if (on_time == 0)
		{
			off_time = 100;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		}
		
	}
	 
	else if (off_time)
	{
		off_time--;
		if (off_time == 0)
		{
			on_time = 100;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		}

	}
	else
	{
		off_time = 100;
	}


}
