#include "magnet.h"
#include "mymath.h"
#include "vector3.h"
#include "filter.h"
MAG_S mag_s;

#define Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))

void MAG_S::Read_Mag_Data(void)
{
	//u8 ak8975_buffer[6]; //接收数据缓存
	static uint8_t mag_filt_cnt = 0;
	if (++mag_filt_cnt > AK8975_filt_num)
	{
		mag_filt_cnt = 0;

	}


	Mag_Adc.y = (Byte16(int16_t, ahrs.buffer[16], ahrs.buffer[15]));    // m.X


	Mag_Adc.x = (Byte16(int16_t, ahrs.buffer[18], ahrs.buffer[17]));   // m.Y

	Mag_Adc.z = -(Byte16(int16_t, ahrs.buffer[20], ahrs.buffer[19]));    // m.Z

	Mag_Adc.x = (int)filter.Moving_Median(0, 5, (float)Mag_Adc.x);
	Mag_Adc.y = (int)filter.Moving_Median(1, 5, (float)Mag_Adc.y);
	Mag_Adc.z = (int)filter.Moving_Median(2, 5, (float)Mag_Adc.z);
	//

	//椭圆校准 //
	//零偏校准 //
	/*	AK_FILT_BUF[mag_filt_cnt].x = (Mag_Adc.x - Mag_Offset.x);
	AK_FILT_BUF[mag_filt_cnt].y = (Mag_Adc.y - Mag_Offset.y);
	AK_FILT_BUF[mag_filt_cnt].z = (Mag_Adc.z - Mag_Offset.z);*/
	//AK_FILT_BUF[mag_filt_cnt] = mag_outcome(AK_FILT_BUF[mag_filt_cnt]);
	AK_FILT_BUF[mag_filt_cnt].x = (Mag_Adc.x);
	AK_FILT_BUF[mag_filt_cnt].y = (Mag_Adc.y);
	AK_FILT_BUF[mag_filt_cnt].z = (Mag_Adc.z);
	AK_FILT_BUF[mag_filt_cnt] = mag_outcome_3D(AK_FILT_BUF[mag_filt_cnt]);
	//AK_FILT_BUF[mag_filt_cnt] = Mag_Adc-Mag_Offset;
	////
	// Mag_Val.x = ( Mag_Adc.x -  Mag_Offset.x) ;
	// Mag_Val.y = ( Mag_Adc.y -  Mag_Offset.y) ;
	// Mag_Val.z = ( Mag_Adc.z -  Mag_Offset.z) ;
	//磁力计中点矫正	

	/* Mag_Val.x = ( Mag_Adc.x -  Mag_Offset.x) ;
	Mag_Val.y = ( Mag_Adc.y -  Mag_Offset.y) ;
	Mag_Val.z = ( Mag_Adc.z -  Mag_Offset.z) ;*/
	CalOffset_Mag();


}


//磁力计中点矫正

void MAG_S::CalOffset_Mag(void)
{
	static Vector3f	MagMAX(-1000, -1000, -1000), MagMIN(1000, 1000, 1000), MagSum;
	static uint16_t cnt_m = 0;

	if (Mag_CALIBRATED)
	{

		if (ABS(Mag_Adc.x)<2000 && ABS(Mag_Adc.y)<2000 && ABS(Mag_Adc.z)<2000)
		{
	           //求最大最小
				MagMAX.x = MAX(Mag_Adc.x, MagMAX.x);
				MagMAX.y = MAX(Mag_Adc.y, MagMAX.y);
				MagMIN.x = MIN(Mag_Adc.x, MagMIN.x);
				MagMIN.y = MIN(Mag_Adc.y, MagMIN.y);	 
				MagMAX.z = MAX(Mag_Adc.z, MagMAX.z);
				MagMIN.z = MIN(Mag_Adc.z, MagMIN.z);
				
				Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);

				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;

				Mag_Gain.y = MagSum.y / MagSum.x;
				Mag_Gain.z = MagSum.z / MagSum.x;

		}


		 

	}
	else
	{

	}
}

void MAG_S::filt_data(void)
{
	uint8_t i;
	Mag_Val.x = 0;
	Mag_Val.y = 0;
	Mag_Val.z = 0;
	for (i = 0; i<AK8975_filt_num; i++)
	{
		Mag_Val.x += AK_FILT_BUF[i].x;
		Mag_Val.y += AK_FILT_BUF[i].y;
		Mag_Val.z += AK_FILT_BUF[i].z;
	}
	Mag_Val.x = Mag_Val.x / AK8975_filt_num;
	Mag_Val.y = Mag_Val.y / AK8975_filt_num;
	Mag_Val.z = Mag_Val.z / AK8975_filt_num;
}

/*椭圆校准函数*/
Vector3f  MAG_S::mag_outcome(Vector3f Mag_Adc_n)
{
	Vector3f Mag_outcome;
	//圆形矫正
	Mag_outcome.x = Mag_Adc_n.x*(my_cos(mag_theta)*my_cos(mag_theta) + mag_s*my_sin(mag_theta)*my_sin(mag_theta)) + Mag_Adc_n.y*(my_cos(mag_theta)*my_sin(mag_theta) - mag_s*my_cos(mag_theta)*my_sin(mag_theta));
	Mag_outcome.y = Mag_Adc_n.y*(mag_s*my_cos(mag_theta)*my_cos(mag_theta) + my_sin(mag_theta)*my_sin(mag_theta)) + Mag_Adc_n.x*(my_cos(mag_theta)*my_sin(mag_theta) - mag_s*my_cos(mag_theta)*my_sin(mag_theta));
	Mag_outcome.z = Mag_Adc_n.z;
	return Mag_outcome;
}
Vector3f MAG_S::mag_outcome_3D(Vector3f Mag_Adc_n)
{

	Vector3f Mag_outcome;
	//3d矫正
	Mag_outcome.x = Mag_Adc_n.x - Mag_Offset.x;
	Mag_outcome.y = (Mag_Adc_n.y - Mag_Offset.y) / Mag_Gain.y;

	Mag_outcome.z = (Mag_Adc_n.z - Mag_Offset.z) / Mag_Gain.z;
	return Mag_outcome;
}