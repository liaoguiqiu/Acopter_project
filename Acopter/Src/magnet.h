#ifndef _MAGNET_H_
#define	_MAGNET_H_
 
#include "stdbool.h"
#include "include.h"
#include "parameter.h"
#include "mymath.h"
#include "AHRS.h"



#define CALIBRATING_MAG_CYCLES              10000  //校准时间持续20s


#define AK8975_filt_num    3
enum AK_ITM
{
	MX = 0,
	MY,
	MZ
};

class MAG_S
{
public:
	Vector3i  Mag_Adc;			//采样值
	Vector3f    Mag_Offset;		//偏移值
	Vector3f 	Mag_Gain;			//比例缩放	
	Vector3f 	Mag_Val;

	u8 ak8975_ok;
	float AK_FILT_BUF[3][(AK8975_filt_num + 1)];

	u8 Mag_CALIBRATED ;


	void Read_Mag_Data();
	void CalOffset_Mag(void);
	void  filt_data(void);
private:

};
 
extern MAG_S mag_s;
  

#endif

