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
		led_state0();

	}
	else
	{
	
		led_state1();

		if (mag_s.Mag_CALIBRATED == 1 
			||ahrs.Acc_CALIBRATE==1
			||ahrs.Gyro_CALIBRATE==1
			||flash_save.save_on==1)
		{
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