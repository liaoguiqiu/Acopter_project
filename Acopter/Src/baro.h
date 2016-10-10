#ifndef __BARO_H
#define __BARO_H 

#include "include.h"
#include "mymath.h"

#define MS5611_ADDR             0x77   //0xee //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096



//ÆøÑ¹¼Æ³õÊ¼»¯
void MS5611_Init(void);
//¶ÁÈ¡ÆøÑ¹¼ÆÊý¾Ý
int MS5611_Update(void);
//·µ»ØÆøÑ¹¸ß¶È

int32_t MS5611_Get_BaroAlt(void);

void MS5611_Reset(void);
u8 MS5611_Read_Prom(void);
void MS5611_Start_T(void);
void MS5611_Start_P(void);
void MS5611_Read_Adc_T(void);
void MS5611_Read_Adc_P(void);
void MS5611_BaroAltCalculate(void);

extern u8 ms5611_ok;
extern int32_t baroAlt, baroAltOld;
extern float baro_alt_speed;
class BARO
{
public:
	 
	BARO()
	{


	}


private:

};

extern BARO baro;

#endif
