#include "baro.h"
#include "delay.h"
#include "i2c_soft.h" 
#include "sys.h"
#include "filter.h"
#include "height_ctrl.h"

BARO baro;
 


#define BARO_CAL_CNT 200

void BARO::MS5611_Reset(void)
{
   i2c_soft.IIC_Write_1Byte(MS5611_ADDR, CMD_RESET, 1);
}

u8 BARO::MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };
	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
		check += i2c_soft.IIC_Read_nByte(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}

	if (check == PROM_NB)
		return 1;
	else
		return 0;
}


void BARO::MS5611_Read_Adc_T(void)
{
	i2c_soft.IIC_Read_nByte(MS5611_ADDR, CMD_ADC_READ, 3, t_rxbuf); // read ADC
}

void  BARO::MS5611_Read_Adc_P(void)
{
	i2c_soft.IIC_Read_nByte(MS5611_ADDR, CMD_ADC_READ, 3, p_rxbuf); // read ADC
}

void  BARO::MS5611_Start_T(void)
{
	i2c_soft.IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1); // D2 (temperature) conversion start!
}

void  BARO::MS5611_Start_P(void)
{
	i2c_soft.IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1); // D1 (pressure) conversion start!
}

u8 ms5611_ok;
void  BARO::MS5611_Init(void)
{

	delay_ms(10);
	//´«¸ÐÆ÷¸´Î»
	MS5611_Reset();
	delay_ms(3);
	ms5611_ok = !(MS5611_Read_Prom());
	//¿ªÊ¼¶ÁÈ¡ÎÂ¶È
	MS5611_Start_T();
}

int  BARO::MS5611_Update(void)
{
	static int state = 0;

	//	I2C_FastMode = 0;

	if (state)
	{
		MS5611_Read_Adc_P();
		MS5611_Start_T();
		MS5611_BaroAltCalculate();
		state = 0;
	}
	else
	{
		MS5611_Read_Adc_T();
		MS5611_Start_P();
		state = 1;
	}
	return (state);
}


void  BARO::MS5611_BaroAltCalculate(void)
{

  float baro_T=	time.Get_Cycle_T(3);
	static u8 baro_start;

	int32_t temperature, off2 = 0, sens2 = 0, delt;
	
	float alt_3;

	int32_t dT;
	int64_t off;
	int64_t sens;

	static int32_t sum_tmp_5611 = 0;
	

	ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
	ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];

	dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
	off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
	sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
	temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);

	if (temperature < 2000) { // temperature lower than 20degC 
		delt = temperature - 2000;
		delt = delt * delt;
		off2 = (5 * delt) >> 1;
		sens2 = (5 * delt) >> 2;
		if (temperature < -1500) { // temperature lower than -15degC
			delt = temperature + 1500;
			delt = delt * delt;
			off2 += 7 * delt;
			sens2 += (11 * delt) >> 1;
		}
	}
	off -= off2;
	sens -= sens2;
	pressure = (((ms5611_up * sens) >> 21) - off) >> 15;
	//pressure = (int)((1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter

	alt_3 = (101000 - pressure) / 1000.0f;
	pressure = (0.0082f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f);
	
	
	// - ( temperature_5611  ) *650 *(pressure/101000.0);

	// 		if( pressure < 101000 )
	// 		{
	// 			pressure = 80 *my_pow( (101000 - pressure)/1000.0 ) + 0.08 *(101000 - pressure)*100.0;
	// 			
	// 		}
	// 		else
	// 		{
	// 			pressure = baro_Offset;
	// 		}

	//if(!(baro_Offset == 0))




	baroAlt = (int)(10 * (0.1f *(pressure - offset))); //cm

	//baro_alt_speed = (baroAlt - baroAltOld)/baro_T; // 20ms Ò»´Î /0.02 = *50 µ¥Î»cm/s
	height_speed_filter(baro_T);
	//baroAltOld = baroAlt;

	if (baro_start < 100)
	{
		CALL_OFFSET = 1;
		baro_start++;
		baro_alt_speed = 0;
		baroAlt = 0;
	}

	 

	temperature_5611 += 0.01f *((0.01f *temperature) - temperature_5611);
	/*----------获得气压计偏移量--------*/
	call_offset();
}

void BARO:: height_speed_filter(float T)
{
  
  	   float high_filed_hz = 3;
	   float speed_filed_hz = 1;
	    

	baroAlt = baroAlt * 10;//mm
	
	//high_filed_lf += (1 / (1 + 1 / (high_filed_hz * 3.14f *T)))*(baroAlt - high_filed_lf);
	//filter.Moving_Average(high_filed, h_filt_buff, BARO_fil_NUM, h_filt_cnt, &high_filed);
	//if (open_kalman)
		//baro_height_pos_acc_kalman.pos_acc_kalman(T, baroAlt, hlt_ctl.wz_acc_mms2);

	/*high_filed = baro_height_pos_acc_kalman.X(0, 0);*/
       // high_filed =high_filed_lf;
	if (pos_wz_kalamn.start_cnt < 200)
	{
		pos_wz_kalamn.start_cnt++;
		pos_wz_kalamn.X = baroAlt;
	}
	else
	{
	}
	pos_wz_kalamn.Kalman_Filter(T, baroAlt, hlt_ctl.wz_speed);


	high_filed = pos_wz_kalamn.X;
	baro_alt_speed = (high_filed - baroAltOld)/T;//mm
	speed_filed += (1 / (1 + 1 / (speed_filed_hz * 3.14f *T)))*(baro_alt_speed - speed_filed);

	//filter.Moving_Average(baro_alt_speed, speed_filt_buff, BARO_speed_fil_NUM, speed_filt_cnt, &speed_filed);
	baroAltOld = high_filed;
}
int32_t  BARO::MS5611_Get_BaroAlt(void)
{
	return baroAlt;
}
void BARO:: call_offset(void)
{
	if (CALL_OFFSET)
	{
		sum_cnt++;
		offset_sum += pressure;
		if (sum_cnt >= BARO_CALL_OFF_SET_TIME)
		{
			offset = offset_sum / BARO_CALL_OFF_SET_TIME;
			sum_cnt = 0;
			CALL_OFFSET = 0;
			offset_sum = 0;
		}

	}

}
