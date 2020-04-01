#ifndef _MAGNET_H_
#define	_MAGNET_H_

#include "stdbool.h"
#include "include.h"
#include "parameter.h"
#include "mymath.h"
#include "AHRS.h"



#define CALIBRATING_MAG_CYCLES              10000  //校准时间持续20s


#define AK8975_filt_num    10
enum AK_ITM
{
	MX = 0,
	MY,
	MZ
};

class MAG_S
{
public:
	MAG_S()
	{
		mag_theta = 0.0;
		mag_s = 1.0;
		//Mag_Offset.x = 627.0f;
		//Mag_Offset.y = 123.0f;
		////Mag_Offset.z=-398;
	}
	float mag_theta;
	float mag_s;
	Vector3i  Mag_Adc;			//采样值
	Vector3f    Mag_Offset;		//偏移值
	Vector3f 	Mag_Gain;			//比例缩放	
	Vector3f 	Mag_Val;

	u8 ak8975_ok;
	//float AK_FILT_BUF[3][(AK8975_filt_num + 1)];
	Vector3f AK_FILT_BUF[(AK8975_filt_num + 1)];
	u8 Mag_CALIBRATED;


	void Read_Mag_Data();
	void CalOffset_Mag(void);
	Vector3f mag_outcome(Vector3f Mag_Adc_n);
	Vector3f mag_outcome_3D(Vector3f Mag_Adc_n);
	void  filt_data(void);

private:

};

extern MAG_S mag_s;


#endif

