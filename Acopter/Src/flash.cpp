
 
#include "FLASH.h"
#include "parameter.h"
#include "ctrl.h"
#include "AHRS.h"
#include "magnet.h"
#include "height_ctrl.h"
#include "delay.h" 

flash_save_para flash_save;
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t GetSector(uint32_t Address);

void FLASHinit(void);






static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
	{
		sector = FLASH_SECTOR_7;
	}
	return sector;
}

void FLASHinit(void)
{
	uint32_t FirstSector = 0, NbOfSectors = 0;

	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
	//NbOfSectors=1;
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;
}


uint8_t flash_save_para::  UpdateTheFLASH(uint32_t Address, uint8_t *buf, short size)
{
	short number = 0;
	u32  SECTORError;
	HAL_FLASH_Unlock();
	FLASHinit();
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		while (1)
		{
			//asm("nop");
			return 0;
		}
	}
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();
	// Address = FLASH_USER_START_ADDR;

	while (number<size)
	{
		number++;

		delay_ms(1);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, *buf) == HAL_OK)
		{
			Address = Address + 1;
			buf++;
		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			User can add here some code to deal with this error */
			while (1)
			{
				//BSP_LED_On(LED3);
				//asm("nop");
				return 0;
			}
		}
	}

	HAL_FLASH_Lock();
	return 1;
}

void flash_save_para:: ReadTheFLASH(uint32_t Address, uint8_t *buf, short size)
{
	short number = 0;

	while (number < size)
	{
		delay_ms(1);
		buf[number] = *(__IO uint8_t *)Address;
		Address = Address + 1;
		number++;
	}
}
void flash_save_para:: flash_byte_to_float(uint8_t * buf, short number, float * paramer)
{
	char byte_float[4];
	float paramer1;
	byte_float[0] = *(buf + 2 + number * 4);
	byte_float[1] = *(buf + 2 + number * 4 + 1);
	byte_float[2] = *(buf + 2 + number * 4 + 2);
	byte_float[3] = *(buf + 2 + number * 4 + 3);
	paramer1 = *((float*)(byte_float));
	*paramer = paramer1;
	//return paramer;
}
void flash_save_para::flash_float_to_byte(uint8_t * buf, short number, float* paramer)
{
	unsigned char *point;
	point = (unsigned char *)paramer; //得到float的地址
	buf[2 + number * 4] = point[0];
	buf[2 + number * 4 + 1] = point[1];
	buf[2 + number * 4 + 2] = point[2];
	buf[2 + number * 4 + 3] = point[3];


}



void flash_save_para::flash_save_parameters(void)
{
	 save_arry[0] = 0xff;
	 save_arry[1] = 0xfe;
	 save_arry[max_flash_save_len - 2] = 0xfe;
	 save_arry[max_flash_save_len - 1] = 0xff;

	flash_float_to_byte((uint8_t *) save_arry,
		(short)0,
		((float*)ahrs.Gyro_CALIBRATE));
	flash_float_to_byte((uint8_t *) save_arry,
		(short)1,
		&ahrs.Gyro_Offset.x);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)2,
		&ahrs.Gyro_Offset.y);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)3,
		&ahrs.Gyro_Offset.z);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)4,
		(float*)ahrs.Acc_CALIBRATE);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)5,
		&ahrs.Acc_Offset.x);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)6,
		&ahrs.Acc_Offset.y);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)7,
		&ahrs.Acc_Offset.z);
	 

	flash_float_to_byte((uint8_t *) save_arry,
		(short)9,
		& mag_s.Mag_Offset.x);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)10,
		&mag_s.Mag_Offset.y);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)11,
		&mag_s.Mag_Offset.z);

	//pid
	flash_float_to_byte((uint8_t *) save_arry,
		(short)12,
		&ctrl_s.ctrl_1.PID[PIDROLL].kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)13,
		&ctrl_s.ctrl_1.PID[PIDROLL].ki);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)14,
		&ctrl_s.ctrl_1.PID[PIDROLL].kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)15,
		&ctrl_s.ctrl_1.PID[PIDROLL].kdamp);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)16,
		&ctrl_s.ctrl_1.PID[PIDPITCH].kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)17,
		&ctrl_s.ctrl_1.PID[PIDPITCH].ki);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)18,
		&ctrl_s.ctrl_1.PID[PIDPITCH].kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)19,
		&ctrl_s.ctrl_1.PID[PIDPITCH].kdamp);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)20,
		&ctrl_s.ctrl_1.PID[PIDYAW].kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)21,
		&ctrl_s.ctrl_1.PID[PIDYAW].ki);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)22,
		&ctrl_s.ctrl_1.PID[PIDYAW].kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)23,
		&ctrl_s.ctrl_1.PID[PIDYAW].kdamp);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)24,
		&ctrl_s.ctrl_2.PID[PIDROLL].kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)25,
		&ctrl_s.ctrl_2.PID[PIDROLL].ki);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)26,
		&ctrl_s.ctrl_2.PID[PIDROLL].kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)27,
		&ctrl_s.ctrl_2.PID[PIDROLL].kdamp);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)28,
		&ctrl_s.ctrl_2.PID[PIDPITCH].kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)29,
		&ctrl_s.ctrl_2.PID[PIDPITCH].ki);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)30,
		&ctrl_s.ctrl_2.PID[PIDPITCH].kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)31,
		&ctrl_s.ctrl_2.PID[PIDPITCH].kdamp);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)32,
		&ctrl_s.ctrl_2.PID[PIDYAW].kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)33,
		&ctrl_s.ctrl_2.PID[PIDYAW].ki);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)34,
		&ctrl_s.ctrl_2.PID[PIDYAW].kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)35,
		&ctrl_s.ctrl_2.PID[PIDYAW].kdamp);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)36,
		&ctrl_s.ctrl_1.FB);
	//图像
	 


	//光流
	 
	//高度
	flash_float_to_byte((uint8_t *) save_arry,
		(short)47,
		&hlt_ctl.wz_speed_pid.kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)48,
		&hlt_ctl.wz_speed_pid.kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)49,
		&hlt_ctl.wz_speed_pid.ki);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)50,
		&hlt_ctl. ultra_pid.kp);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)51,
		&hlt_ctl.ultra_pid.kd);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)52,
		&hlt_ctl.ultra_pid.ki);

	flash_float_to_byte((uint8_t *) save_arry,
		(short)53,
		&ctrl_s.ctrl_angle_offset.x);
	flash_float_to_byte((uint8_t *) save_arry,
		(short)54,
		&ctrl_s.ctrl_angle_offset.y);

	//舵机零位与正式限幅
	 

	//        static short arry_size;
	//        arry_size = sizeof( save_arry);
	UpdateTheFLASH(FLASH_USER_START_ADDR, (uint8_t *) save_arry, sizeof( save_arry));

}

uint8_t flash_save_para:: flash_read_parameters(void)
{
	ReadTheFLASH(FLASH_USER_START_ADDR, (uint8_t *) save_arry, sizeof( save_arry));



	if ( save_arry[0] == 0xff &&
		 save_arry[1] == 0xfe &&
		 save_arry[max_flash_save_len - 2] == 0xfe &&
		 save_arry[max_flash_save_len - 1] == 0xff)
	{


		flash_byte_to_float((uint8_t *) save_arry,
			(short)0,
			((float*)ahrs.Gyro_CALIBRATE));
		flash_byte_to_float((uint8_t *) save_arry,
			(short)1,
			&ahrs.Gyro_Offset.x);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)2,
			&ahrs.Gyro_Offset.y);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)3,
			&ahrs.Gyro_Offset.z);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)4,
			(float*)ahrs.Acc_CALIBRATE);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)5,
			&ahrs.Acc_Offset.x);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)6,
			&ahrs.Acc_Offset.y);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)7,
			&ahrs.Acc_Offset.z);
		 
		flash_byte_to_float((uint8_t *) save_arry,
			(short)9,
			&mag_s.Mag_Offset.x);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)10,
			&mag_s.Mag_Offset.y);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)11,
			&mag_s.Mag_Offset.z);

		//pid
		flash_byte_to_float((uint8_t *) save_arry,
			(short)12,
			&ctrl_s.ctrl_1.PID[PIDROLL].kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)13,
			&ctrl_s.ctrl_1.PID[PIDROLL].ki);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)14,
			&ctrl_s.ctrl_1.PID[PIDROLL].kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)15,
			&ctrl_s.ctrl_1.PID[PIDROLL].kdamp);

		flash_byte_to_float((uint8_t *) save_arry,
			(short)16,
			&ctrl_s.ctrl_1.PID[PIDPITCH].kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)17,
			&ctrl_s.ctrl_1.PID[PIDPITCH].ki);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)18,
			&ctrl_s.ctrl_1.PID[PIDPITCH].kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)19,
			&ctrl_s.ctrl_1.PID[PIDPITCH].kdamp);

		flash_byte_to_float((uint8_t *) save_arry,
			(short)20,
			&ctrl_s.ctrl_1.PID[PIDYAW].kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)21,
			&ctrl_s.ctrl_1.PID[PIDYAW].ki);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)22,
			&ctrl_s.ctrl_1.PID[PIDYAW].kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)23,
			&ctrl_s.ctrl_1.PID[PIDYAW].kdamp);

		flash_byte_to_float((uint8_t *) save_arry,
			(short)24,
			&ctrl_s.ctrl_2.PID[PIDROLL].kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)25,
			&ctrl_s.ctrl_2.PID[PIDROLL].ki);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)26,
			&ctrl_s.ctrl_2.PID[PIDROLL].kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)27,
			&ctrl_s.ctrl_2.PID[PIDROLL].kdamp);

		flash_byte_to_float((uint8_t *) save_arry,
			(short)28,
			&ctrl_s.ctrl_2.PID[PIDPITCH].kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)29,
			&ctrl_s.ctrl_2.PID[PIDPITCH].ki);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)30,
			&ctrl_s.ctrl_2.PID[PIDPITCH].kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)31,
			&ctrl_s.ctrl_2.PID[PIDPITCH].kdamp);

		flash_byte_to_float((uint8_t *) save_arry,
			(short)32,
			&ctrl_s.ctrl_2.PID[PIDYAW].kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)33,
			&ctrl_s.ctrl_2.PID[PIDYAW].ki);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)34,
			&ctrl_s.ctrl_2.PID[PIDYAW].kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)35,
			&ctrl_s.ctrl_2.PID[PIDYAW].kdamp);

		flash_byte_to_float((uint8_t *) save_arry,
			(short)36,
			&ctrl_s.ctrl_1.FB);
		//图像
	 

		//光流
		 

		//高度
		flash_byte_to_float((uint8_t *) save_arry,
			(short)47,
			&hlt_ctl.wz_speed_pid.kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)48,
			&hlt_ctl.wz_speed_pid.kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)49,
			&hlt_ctl.wz_speed_pid.ki);

		flash_byte_to_float((uint8_t *) save_arry,
			(short)50,
			 &hlt_ctl. ultra_pid.kp);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)51,
			&hlt_ctl.ultra_pid.kd);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)52,
			&hlt_ctl.ultra_pid.ki);


		flash_byte_to_float((uint8_t *) save_arry,
			(short)53,
			&ctrl_s. ctrl_angle_offset.x);
		flash_byte_to_float((uint8_t *) save_arry,
			(short)54,
			&ctrl_s.ctrl_angle_offset.y);
		 
		return 1;
	}
	else
	{
		return 0;
	}

}