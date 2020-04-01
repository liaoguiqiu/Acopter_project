#ifndef _AHRS_H
#define _AHRS_H

//#include "stm32f4xx.h"

#include "include.h"
#include "string.h"
#include "parameter.h"
 



#define GYRO_DEATH_ZOOM 20   //  20 / 65536

#define OFFSET_AV_NUM 1000
#define FILTER_NUM_A 10
#define FILTER_NUM_G 10

/*---------mpu驱动----------*/


#define MPU9250_ON  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET)
#define MPU9250_OFF HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET)

#define MPU9250_INT_PIN (GPIO_PIN_4)

/*----Sensitivity--------------------------------------------------------*/

#define MPU9250A_2g       ((float)0.000061035156f)  // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.00122070312f)  // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f)  // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f)  // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f)  // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f)  // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f)  // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f)  // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   ((float)0.6f)             // 0.6 uT/LSB

#define MPU9250T_85degC   ((float)0.002995177763f)  // 0.002995177763 degC/LSB

/* ---- MPU6500 Reg In MPU9250 ---------------------------------------------- */

#define MPU6500_I2C_ADDR            ((uint8_t)0xD0)
#define MPU6500_Device_ID           ((uint8_t)0x71)  // In MPU9250

#define MPU6500_SELF_TEST_XG        ((uint8_t)0x00)
#define MPU6500_SELF_TEST_YG        ((uint8_t)0x01)
#define MPU6500_SELF_TEST_ZG        ((uint8_t)0x02)
#define MPU6500_SELF_TEST_XA        ((uint8_t)0x0D)
#define MPU6500_SELF_TEST_YA        ((uint8_t)0x0E)
#define MPU6500_SELF_TEST_ZA        ((uint8_t)0x0F)
#define MPU6500_XG_OFFSET_H         ((uint8_t)0x13)
#define MPU6500_XG_OFFSET_L         ((uint8_t)0x14)
#define MPU6500_YG_OFFSET_H         ((uint8_t)0x15)
#define MPU6500_YG_OFFSET_L         ((uint8_t)0x16)
#define MPU6500_ZG_OFFSET_H         ((uint8_t)0x17)
#define MPU6500_ZG_OFFSET_L         ((uint8_t)0x18)
#define MPU6500_SMPLRT_DIV          ((uint8_t)0x19)
#define MPU6500_CONFIG              ((uint8_t)0x1A)
#define MPU6500_GYRO_CONFIG         ((uint8_t)0x1B)
#define MPU6500_ACCEL_CONFIG        ((uint8_t)0x1C)
#define MPU6500_ACCEL_CONFIG_2      ((uint8_t)0x1D)
#define MPU6500_LP_ACCEL_ODR        ((uint8_t)0x1E)
#define MPU6500_MOT_THR             ((uint8_t)0x1F)
#define MPU6500_FIFO_EN             ((uint8_t)0x23)
#define MPU6500_I2C_MST_CTRL        ((uint8_t)0x24)
#define MPU6500_I2C_SLV0_ADDR       ((uint8_t)0x25)
#define MPU6500_I2C_SLV0_REG        ((uint8_t)0x26)
#define MPU6500_I2C_SLV0_CTRL       ((uint8_t)0x27)
#define MPU6500_I2C_SLV1_ADDR       ((uint8_t)0x28)
#define MPU6500_I2C_SLV1_REG        ((uint8_t)0x29)
#define MPU6500_I2C_SLV1_CTRL       ((uint8_t)0x2A)
#define MPU6500_I2C_SLV2_ADDR       ((uint8_t)0x2B)
#define MPU6500_I2C_SLV2_REG        ((uint8_t)0x2C)
#define MPU6500_I2C_SLV2_CTRL       ((uint8_t)0x2D)
#define MPU6500_I2C_SLV3_ADDR       ((uint8_t)0x2E)
#define MPU6500_I2C_SLV3_REG        ((uint8_t)0x2F)
#define MPU6500_I2C_SLV3_CTRL       ((uint8_t)0x30)
#define MPU6500_I2C_SLV4_ADDR       ((uint8_t)0x31)
#define MPU6500_I2C_SLV4_REG        ((uint8_t)0x32)
#define MPU6500_I2C_SLV4_DO         ((uint8_t)0x33)
#define MPU6500_I2C_SLV4_CTRL       ((uint8_t)0x34)
#define MPU6500_I2C_SLV4_DI         ((uint8_t)0x35)
#define MPU6500_I2C_MST_STATUS      ((uint8_t)0x36)
#define MPU6500_INT_PIN_CFG         ((uint8_t)0x37)
#define MPU6500_INT_ENABLE          ((uint8_t)0x38)
#define MPU6500_INT_STATUS          ((uint8_t)0x3A)
#define MPU6500_ACCEL_XOUT_H        ((uint8_t)0x3B)
#define MPU6500_ACCEL_XOUT_L        ((uint8_t)0x3C)
#define MPU6500_ACCEL_YOUT_H        ((uint8_t)0x3D)
#define MPU6500_ACCEL_YOUT_L        ((uint8_t)0x3E)
#define MPU6500_ACCEL_ZOUT_H        ((uint8_t)0x3F)
#define MPU6500_ACCEL_ZOUT_L        ((uint8_t)0x40)
#define MPU6500_TEMP_OUT_H          ((uint8_t)0x41)
#define MPU6500_TEMP_OUT_L          ((uint8_t)0x42)
#define MPU6500_GYRO_XOUT_H         ((uint8_t)0x43)
#define MPU6500_GYRO_XOUT_L         ((uint8_t)0x44)
#define MPU6500_GYRO_YOUT_H         ((uint8_t)0x45)
#define MPU6500_GYRO_YOUT_L         ((uint8_t)0x46)
#define MPU6500_GYRO_ZOUT_H         ((uint8_t)0x47)
#define MPU6500_GYRO_ZOUT_L         ((uint8_t)0x48)
#define MPU6500_EXT_SENS_DATA_00    ((uint8_t)0x49)
#define MPU6500_EXT_SENS_DATA_01    ((uint8_t)0x4A)
#define MPU6500_EXT_SENS_DATA_02    ((uint8_t)0x4B)
#define MPU6500_EXT_SENS_DATA_03    ((uint8_t)0x4C)
#define MPU6500_EXT_SENS_DATA_04    ((uint8_t)0x4D)
#define MPU6500_EXT_SENS_DATA_05    ((uint8_t)0x4E)
#define MPU6500_EXT_SENS_DATA_06    ((uint8_t)0x4F)
#define MPU6500_EXT_SENS_DATA_07    ((uint8_t)0x50)
#define MPU6500_EXT_SENS_DATA_08    ((uint8_t)0x51)
#define MPU6500_EXT_SENS_DATA_09    ((uint8_t)0x52)
#define MPU6500_EXT_SENS_DATA_10    ((uint8_t)0x53)
#define MPU6500_EXT_SENS_DATA_11    ((uint8_t)0x54)
#define MPU6500_EXT_SENS_DATA_12    ((uint8_t)0x55)
#define MPU6500_EXT_SENS_DATA_13    ((uint8_t)0x56)
#define MPU6500_EXT_SENS_DATA_14    ((uint8_t)0x57)
#define MPU6500_EXT_SENS_DATA_15    ((uint8_t)0x58)
#define MPU6500_EXT_SENS_DATA_16    ((uint8_t)0x59)
#define MPU6500_EXT_SENS_DATA_17    ((uint8_t)0x5A)
#define MPU6500_EXT_SENS_DATA_18    ((uint8_t)0x5B)
#define MPU6500_EXT_SENS_DATA_19    ((uint8_t)0x5C)
#define MPU6500_EXT_SENS_DATA_20    ((uint8_t)0x5D)
#define MPU6500_EXT_SENS_DATA_21    ((uint8_t)0x5E)
#define MPU6500_EXT_SENS_DATA_22    ((uint8_t)0x5F)
#define MPU6500_EXT_SENS_DATA_23    ((uint8_t)0x60)
#define MPU6500_I2C_SLV0_DO         ((uint8_t)0x63)
#define MPU6500_I2C_SLV1_DO         ((uint8_t)0x64)
#define MPU6500_I2C_SLV2_DO         ((uint8_t)0x65)
#define MPU6500_I2C_SLV3_DO         ((uint8_t)0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  ((uint8_t)0x67)
#define MPU6500_SIGNAL_PATH_RESET   ((uint8_t)0x68)
#define MPU6500_MOT_DETECT_CTRL     ((uint8_t)0x69)
#define MPU6500_USER_CTRL           ((uint8_t)0x6A)
#define MPU6500_PWR_MGMT_1          ((uint8_t)0x6B)
#define MPU6500_PWR_MGMT_2          ((uint8_t)0x6C)
#define MPU6500_FIFO_COUNTH         ((uint8_t)0x72)
#define MPU6500_FIFO_COUNTL         ((uint8_t)0x73)
#define MPU6500_FIFO_R_W            ((uint8_t)0x74)
#define MPU6500_WHO_AM_I            ((uint8_t)0x75)	// ID = 0x71 In MPU9250
#define MPU6500_XA_OFFSET_H         ((uint8_t)0x77)
#define MPU6500_XA_OFFSET_L         ((uint8_t)0x78)
#define MPU6500_YA_OFFSET_H         ((uint8_t)0x7A)
#define MPU6500_YA_OFFSET_L         ((uint8_t)0x7B)
#define MPU6500_ZA_OFFSET_H         ((uint8_t)0x7D)
#define MPU6500_ZA_OFFSET_L         ((uint8_t)0x7E)

#define MPU6500_I2C_SLVx_EN         ((uint8_t)0x80)
#define MPU6500_I2C_SLV4_DONE       ((uint8_t)0x40)
#define MPU6500_I2C_SLV4_NACK       ((uint8_t)0x10)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             ((uint8_t)0x0C)
#define AK8963_Device_ID            ((uint8_t)0x48)

// Read-only Reg
#define AK8963_WIA                  ((uint8_t)0x00)
#define AK8963_INFO                 ((uint8_t)0x01)
#define AK8963_ST1                  ((uint8_t)0x02)
#define AK8963_HXL                  ((uint8_t)0x03)
#define AK8963_HXH                  ((uint8_t)0x04)
#define AK8963_HYL                  ((uint8_t)0x05)
#define AK8963_HYH                  ((uint8_t)0x06)
#define AK8963_HZL                  ((uint8_t)0x07)
#define AK8963_HZH                  ((uint8_t)0x08)
#define AK8963_ST2                  ((uint8_t)0x09)
// Write/Read Reg
#define AK8963_CNTL1                ((uint8_t)0x0A)
#define AK8963_CNTL2                ((uint8_t)0x0B)
#define AK8963_ASTC                 ((uint8_t)0x0C)
#define AK8963_TS1                  ((uint8_t)0x0D)
#define AK8963_TS2                  ((uint8_t)0x0E)
#define AK8963_I2CDIS               ((uint8_t)0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 ((uint8_t)0x10)
#define AK8963_ASAY                 ((uint8_t)0x11)
#define AK8963_ASAZ                 ((uint8_t)0x12)
// Status 
#define AK8963_STATUS_DRDY          ((uint8_t)0x01)
#define AK8963_STATUS_DOR           ((uint8_t)0x02)
#define AK8963_STATUS_HOFL          ((uint8_t)0x08)



enum
{
	DIR_X = 0,
	DIR_Y,
	DIR_Z

};

class AHRS_S
{
public:
	 
	 
	char mpu_data_ok;

	int sum_temp[7];
	  uint8_t buffer[28];//9250原始数据      
	  int16_t RAWData[11];
	  short  gyro_sum_cnt;
	  short acc_sum_cnt;
	char Acc_CALIBRATE;
	char Gyro_CALIBRATE;
	char Cali_3d;


	short filter_cnt_gyro;
	short filter_cnt;
	Vector3i Acc_I16;
	Vector3i Gyro_I16;
	float ahrs_tmp[ITEMS];
	Vector3f Acc;
	Vector3f Gyro;

	float FILT_BUF_A[3][(FILTER_NUM_A + 1)];
	float FILT_BUF_G[3][(FILTER_NUM_G + 1)];

	//	XYZ_STRUCT Acc_deg;
	Vector3f Gyro_deg;

	Vector3f Acc_Offset;
	Vector3f Gyro_Offset;

	float mpu_fil_tmp[ITEMS];
 
	/*-------构造--------*/
	AHRS_S()
	{
		mpu_data_ok = 0;
	
	}

   /*-------方法--------*/

	void Transform(float itx, float ity, float itz, float *it_x, float *it_y, float *it_z);

	void magGainCalc(float *max, float *min, float *gain);
	void MPU_READ_INITIAL_DATA(void);
	void MPU9250_Init(SPI_HandleTypeDef *hspi);
	uint8_t MPU9250_Check(void);
	void AHRS_Task(void *parg);


	void ahrs_Data_Prepare(float T);
	void ahrs_Data_Offset(); //校准函数
	void initial_data_filed();
	void ahrs_data_filt(float T);
        
         void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
private:

};
 
extern AHRS_S ahrs;
 
#endif
