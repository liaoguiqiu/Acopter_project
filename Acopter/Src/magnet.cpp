#include "magnet.h"
#include "mymath.h"
MAG_S mag_s;

#define Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))

void MAG_S::  Read_Mag_Data(void)
{
	int16_t mag_temp[3];
	//u8 ak8975_buffer[6]; //接收数据缓存
	static uint8_t mag_filt_cnt = 0;
	if (++mag_filt_cnt > AK8975_filt_num)
	{
		mag_filt_cnt = 0;

	}

	//IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXL,&ak8975_buffer[0]); 
	//IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HXH,&ak8975_buffer[1]);
	mag_temp[1] = (Byte16(int16_t, ahrs.buffer[16], ahrs.buffer[15]));    // m.X

	////IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYL,&ak8975_buffer[2]);
	////IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HYH,&ak8975_buffer[3]);
	mag_temp[0] = (Byte16(int16_t, ahrs.buffer[18], ahrs.buffer[17]));   // m.Y
	////IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZL,&ak8975_buffer[4]);
	////IIC_Read_1Byte(AK8975_ADDRESS,AK8975_HZH,&ak8975_buffer[5]);
	mag_temp[2] = -(Byte16(int16_t, ahrs.buffer[20], ahrs.buffer[19]));    // m.Z
	//
	// 
	AK_FILT_BUF[MX][mag_filt_cnt] = (mag_temp[0] -  Mag_Offset.x);
	AK_FILT_BUF[MY][mag_filt_cnt] = (mag_temp[1] -  Mag_Offset.y);
	AK_FILT_BUF[MZ][mag_filt_cnt] = (mag_temp[2] -  Mag_Offset.z);
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

void MAG_S:: CalOffset_Mag(void)
{
	static Vector3f	MagMAX( -1000, -1000, -1000 ), MagMIN(1000, 1000, 1000 ), MagSum;
	static uint16_t cnt_m = 0;

	if (Mag_CALIBRATED)
	{

		if (ABS( Mag_Adc.x)<1000 && ABS( Mag_Adc.y)<1000 && ABS( Mag_Adc.z)<1000)
		{
			MagMAX.x = MAX( Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX( Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX( Mag_Adc.z, MagMAX.z);

			MagMIN.x = MIN( Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN( Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN( Mag_Adc.z, MagMIN.z);

			if (cnt_m >= CALIBRATING_MAG_CYCLES)
			{
				 Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				 Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				// Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);

				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;

				 Mag_Gain.y = MagSum.x / MagSum.y;
				 Mag_Gain.z = MagSum.x / MagSum.z;

				 
				cnt_m = 0;
				Mag_CALIBRATED = 0;
			}
		}
		cnt_m++;

	}
	else
	{

	}
}

void MAG_S::  filt_data(void)
{
	uint8_t i;
	 Mag_Val.x = 0;
	 Mag_Val.y = 0;
	 Mag_Val.z = 0;
	for (i = 0; i<AK8975_filt_num; i++)
	{



		 Mag_Val.x += AK_FILT_BUF[MX][i];
		 Mag_Val.y += AK_FILT_BUF[MY][i];
		 Mag_Val.z += AK_FILT_BUF[MZ][i];

	}
	 Mag_Val.x =  Mag_Val.x / AK8975_filt_num;
	 Mag_Val.y =  Mag_Val.y / AK8975_filt_num;
	 Mag_Val.z =  Mag_Val.z / AK8975_filt_num;



}