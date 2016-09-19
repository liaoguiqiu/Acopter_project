#include "AHRS.h"

AHRS_S ahrs;

SPI_HandleTypeDef *MPU9250_Handler;


#define MAG_READ_DELAY 30

static void simpdelay(void) {
	for (int i = 0; i < 84000; i++) {
		asm("nop");
	}
}



uint8_t MPU9250_ReadReg(uint8_t ReadAddr) {
	MPU9250_ON;
	uint8_t ReadData = 0;
	uint8_t tx = ReadAddr | 0x80;
	HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MPU9250_Handler, &ReadData, 1, 100);
	MPU9250_OFF;
	return ReadData;
}

void MPU9250_WriteReg(uint8_t WriteAddr, uint8_t WriteData) {
	MPU9250_ON;
	HAL_SPI_Transmit(MPU9250_Handler, &WriteAddr, 1, 100);
	HAL_SPI_Transmit(MPU9250_Handler, &WriteData, 1, 100);
	MPU9250_OFF;
}

void MPU9250_ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes) {
	MPU9250_ON;
	uint8_t tx = ReadAddr | 0x80;
	HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MPU9250_Handler, ReadBuf, Bytes, 100);
	MPU9250_OFF;
}

void MPU9250_Mag_WriteReg(uint8_t writeAddr, uint8_t writeData) {
	uint8_t  status = 0;
	uint32_t timeout = MAG_READ_DELAY;

	MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, writeAddr);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_DO, writeData);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
	simpdelay();

	do {
		status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
		simpdelay();
	} while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
	simpdelay();
}

uint8_t MPU9250_Mag_ReadReg(uint8_t readAddr) {
	uint8_t status = 0;
	uint8_t readData = 0;
	uint32_t timeout = MAG_READ_DELAY;

	MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, readAddr);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
	simpdelay();

	do {
		status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
		simpdelay();
	} while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

	readData = MPU9250_ReadReg(MPU6500_I2C_SLV4_DI);
	simpdelay();
	return readData;
}


void MPU9250_Mag_ReadRegs(uint8_t readAddr, uint8_t *readData, uint8_t lens) {
	for (uint8_t i = 0; i < lens; i++) {
		readData[i] = MPU9250_Mag_ReadReg(readAddr + i);
		simpdelay();
	}
}




#define AK8963_CNTL1_Value 0x16
uint8_t MPU9250_Mag_Init(void) {
	uint8_t buf;
	while (1) {
		MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0C);  //write I2C addr 
		simpdelay();
		MPU9250_WriteReg(MPU6500_I2C_SLV0_DO, AK8963_CNTL1_Value);
		MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL1);     // Set Write Reg
		MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Write, 1 bytes
		// read back to check
		simpdelay();
		MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x8C);//read I2C addr   
		MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL1);     // Set Write Reg
		MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Read, 6 bytes
		simpdelay();
		buf = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_00);   // Read Data
		if (buf != AK8963_CNTL1_Value) {
			asm("nop");
			//return ERROR;
		}
		else
			return SUCCESS;
	}
}





#define MPU9250_InitRegNum 10
void AHRS_S:: MPU9250_Init(SPI_HandleTypeDef *hspi) {
	MPU9250_Handler = hspi;
	uint8_t i = 0;
	/*
	uint8_t MPU6500_InitData[MPU9250_InitRegNum][2] =    {
	{ 0x80, MPU6500_PWR_MGMT_1 },     // Reset Device
	{ 0x04, MPU6500_PWR_MGMT_1 },     // Clock Source
	{ 0x10, MPU6500_INT_PIN_CFG },    // Set INT_ANYRD_2CLEAR
	{ 0x01, MPU6500_INT_ENABLE },     // Set RAW_RDY_EN
	{ 0x00, MPU6500_PWR_MGMT_2 },     // Enable Acc & Gyro
	{ 0x00, MPU6500_SMPLRT_DIV },     // Sample Rate Divider
	{ 0x13, MPU6500_GYRO_CONFIG },    // default : +-1000dps
	{ 0x08, MPU6500_ACCEL_CONFIG },   // default : +-4G
	{ 0x07, MPU6500_CONFIG },         // default : LPS_3600Hz
	{ 0x0B, MPU6500_ACCEL_CONFIG_2 }, // default : LPS_41Hz 1011
	{ 0x30, MPU6500_USER_CTRL },      // Set I2C_MST_EN, I2C_IF_DIS
	};*/
	uint8_t MPU6500_InitData[MPU9250_InitRegNum][2] = {
		//{ 0x80, MPU6500_PWR_MGMT_1 },     // Reset Device
		{ 0x04, MPU6500_PWR_MGMT_1 },     // Clock Source
		{ 0x10, MPU6500_INT_PIN_CFG },    // Set INT_ANYRD_2CLEAR
		{ 0x01, MPU6500_INT_ENABLE },     // Set RAW_RDY_EN
		{ 0x00, MPU6500_PWR_MGMT_2 },     // Enable Acc & Gyro
		{ 0x13, MPU6500_GYRO_CONFIG },    // default : +-1000dps
		{ 0x08, MPU6500_ACCEL_CONFIG },   // default : +-4G
		{ 0x07, MPU6500_CONFIG },         // default : LPS_20Hz
		{ 0x0E, MPU6500_ACCEL_CONFIG_2 }, // default : LPS_10Hz 1101
		{ 0x40, MPU6500_I2C_MST_CTRL },
		{ 0x35, MPU6500_USER_CTRL }, // Set I2C_MST_EN, I2C_IF_DIS
	};

	for (i = 0; i < MPU9250_InitRegNum; i++) {
		MPU9250_WriteReg(MPU6500_InitData[i][1], MPU6500_InitData[i][0]);
		simpdelay();
	}
	//check write
	// 	for (i = 1; i < MPU9250_InitRegNum;i++)
	// 	{
	// 		uint8_t temp = MPU9250_ReadReg(MPU6500_InitData[i][1]);
	// 		if (temp!=MPU6500_InitData[i][0])
	// 		{
	// 			while (1)
	// 			{
	// 				asm("nop");
	// 			}
	// 		}
	// 	}
}

uint8_t AHRS_S:: MPU9250_Check(void) {
	uint8_t DeviceID = 0x00;

	/* MPU6500 Check*/
	DeviceID = 0x00;
	DeviceID = MPU9250_ReadReg(MPU6500_WHO_AM_I);
	if (DeviceID != MPU6500_Device_ID)
		return ERROR;

	/* AK8975 Check */
	DeviceID = 0x00;
	DeviceID = MPU9250_Mag_ReadReg(AK8963_WIA);
	if (DeviceID != AK8963_Device_ID) {
		//return ERROR;
	}
	uint8_t tmpRead[3];

	MPU9250_Mag_WriteReg(AK8963_CNTL2, 0x01);       // Reset Device
	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x1F);       // Fuse ROM access mode

	MPU9250_Mag_ReadRegs(AK8963_ASAX, tmpRead, 3);  // Read sensitivity adjustment values
	simpdelay();
	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
	simpdelay();

	if (tmpRead[0] == 0x00 || tmpRead[1] == 0x00 || tmpRead[2] == 0x00) {
		return ERROR;
	}



	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x16);       // 连续测量模式2
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, 0x09); //关闭slv4,陀螺仪加速度计odr=1000,延迟9个周期,磁力计50Hz(磁力计ODR=8Hz)
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_MST_DELAY_CTRL, 0x81); //开启延迟
	simpdelay();


	MPU9250_WriteReg(MPU6500_I2C_MST_CTRL, 0x5D);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ST1);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);//从st1开始读8个字节,中间六个为磁场数据,最后是st2
	simpdelay();


	return SUCCESS;
}
void AHRS_S:: MPU_READ_INITIAL_DATA(void)
{


	HAL_StatusTypeDef status;
	uint8_t tx = MPU6500_ACCEL_XOUT_H | 0x80;
	//HAL_StatusTypeDef status;

	MPU9250_ON;
	status = HAL_SPI_Transmit(&hspi3, &tx, 1, 100);
	if (status == HAL_BUSY)
	{
		HAL_SPI_DMAStop(&hspi3);
		asm("nop");
	}
	if (status != HAL_OK)
	{
		asm("nop");
		return;
	}


	// status = HAL_SPI_Receive(&hspi3, buffer, 22,100);
	if (mpu_data_ok == 0)
		status = HAL_SPI_Receive_DMA(&hspi3, buffer, 22);

	if (status != HAL_OK) {
		asm("nop");
		//OSMemPut(&AHRS_RAWDataPartition, buffer, &err);
		return;
	}
	//buffer16=(short *)buffer;
	// MPU9250_OFF;
	return;



}

void AHRS_S:: Transform(float itx, float ity, float itz, float *it_x, float *it_y, float *it_z)
{
	*it_x = itx;
	*it_y = ity;
	*it_z = itz;

}

#define ACC_ADJ_EN 1

void AHRS_S:: ahrs_Data_Offset()
{
	{
#if  ACC_ADJ_EN
		//Gyro_CALIBRATE=1;
		//Acc_CALIBRATE =  1;

		if ( Acc_CALIBRATE == 1)
		{
			acc_sum_cnt++;
			sum_temp[A_X] += Acc_I16.x;
			sum_temp[A_Y] += Acc_I16.y;
			sum_temp[A_Z] += Acc_I16.z - 65536 / 8;   // +-8G
			//sum_temp[TEM] += Tempreature;

			if (acc_sum_cnt >= OFFSET_AV_NUM)
			{


				Acc_Offset.x = sum_temp[A_X] / OFFSET_AV_NUM;
				Acc_Offset.y = sum_temp[A_Y] / OFFSET_AV_NUM;
				Acc_Offset.z = sum_temp[A_Z] / OFFSET_AV_NUM;
				//Acc_Temprea_Offset = sum_temp[TEM] / OFFSET_AV_NUM;
				acc_sum_cnt = 0;
				Acc_CALIBRATE = 0;

				//Param_SaveAccelOffset(&Acc_Offset);
				sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
			}
		}

#endif

		if (Gyro_CALIBRATE)
		{
			gyro_sum_cnt++;
			sum_temp[G_X] += Gyro_I16.x;
			sum_temp[G_Y] += Gyro_I16.y;
			sum_temp[G_Z] += Gyro_I16.z;
			//sum_temp[TEM] += Tempreature;

			if (gyro_sum_cnt >= OFFSET_AV_NUM)
			{

			 
				Gyro_Offset.x = (float)sum_temp[G_X] / OFFSET_AV_NUM;
				Gyro_Offset.y = (float)sum_temp[G_Y] / OFFSET_AV_NUM;
				Gyro_Offset.z = (float)sum_temp[G_Z] / OFFSET_AV_NUM;
			//	Gyro_Temprea_Offset = sum_temp[TEM] / OFFSET_AV_NUM;
				gyro_sum_cnt = 0;
			//	SendGyroCalData();
			 
				Gyro_CALIBRATE = 0;
				sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
			}
		}
	}

}
#define Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))

void AHRS_S:: ahrs_Data_Prepare(float T)
{
	//	float auto_offset_temp[3];


	ahrs_Data_Offset(); //校准函数


	Acc_I16.x = (Byte16(int16_t, buffer[0], buffer[1]));    // Acc.X
	Acc_I16.y = (Byte16(int16_t, buffer[2], buffer[3]));    // Acc.Y
	Acc_I16.z = (Byte16(int16_t, buffer[4], buffer[5]));    // Acc.Z

	Gyro_I16.x = (Byte16(int16_t, buffer[8], buffer[9]));    // Gyr.X
	Gyro_I16.y = (Byte16(int16_t, buffer[10], buffer[11]));   // Gyr.Y
	Gyro_I16.z = (Byte16(int16_t, buffer[12], buffer[13]));    // Gyr.Z


	//Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]); //tempreature
	//TEM_LPF += 2 *3.14f *T *(Tempreature - TEM_LPF);
	//Ftempreature = TEM_LPF/340.0f + 36.5f;

	//======================================================================
	if (++filter_cnt_gyro > FILTER_NUM_G)
	{
		filter_cnt_gyro = 0;

	}


	if (++filter_cnt > FILTER_NUM_A)
	{
		filter_cnt = 0;
	 
	}
 
	//10 170 4056
	/* 得出校准后的数据 */
	ahrs_tmp[A_X] = (Acc_I16.x - Acc_Offset.x);
	ahrs_tmp[A_Y] = (Acc_I16.y - Acc_Offset.y);
	ahrs_tmp[A_Z] = (Acc_I16.z - Acc_Offset.z);
	ahrs_tmp[G_X] = Gyro_I16.x - Gyro_Offset.x;//
	ahrs_tmp[G_Y] = Gyro_I16.y - Gyro_Offset.y;//
	ahrs_tmp[G_Z] = Gyro_I16.z - Gyro_Offset.z;//


	/* 更新滤波滑动窗口数组 */
	FILT_BUF_A[DIR_X][filter_cnt] = ahrs_tmp[A_X];
	FILT_BUF_A[DIR_Y][filter_cnt] = ahrs_tmp[A_Y];
	FILT_BUF_A[DIR_Z][filter_cnt] = ahrs_tmp[A_Z];
	FILT_BUF_G[DIR_X][filter_cnt_gyro] = ahrs_tmp[G_X];
	FILT_BUF_G[DIR_Y][filter_cnt_gyro] = ahrs_tmp[G_Y];
	FILT_BUF_G[DIR_Z][filter_cnt_gyro] = ahrs_tmp[G_Z];



	//======================================================================
}

void  AHRS_S:: ahrs_data_filt(float T)
{
	u8 i;
	s32 FILT_TMP[ITEMS] = { 0, 0, 0, 0, 0, 0, 0 };
	 
	for (i = 0; i<FILTER_NUM_A; i++)
	{
		      
		         
		FILT_TMP[A_X] += FILT_BUF_A[DIR_X][i];
		FILT_TMP[A_Y] += FILT_BUF_A[DIR_Y][i];
		FILT_TMP[A_Z] += FILT_BUF_A[DIR_Z][i];

	}


	mpu_fil_tmp[A_X] = (float)(FILT_TMP[A_X]) / (float)FILTER_NUM_A;
	mpu_fil_tmp[A_Y] = (float)(FILT_TMP[A_Y]) / (float)FILTER_NUM_A;
	mpu_fil_tmp[A_Z] = (float)(FILT_TMP[A_Z]) / (float)FILTER_NUM_A;
	 
	for (i = 0; i<FILTER_NUM_G; i++)
	{



		FILT_TMP[G_X] += FILT_BUF_G[DIR_X][i];
		FILT_TMP[G_Y] += FILT_BUF_G[DIR_Y][i];
		FILT_TMP[G_Z] += FILT_BUF_G[DIR_Z][i];
	}
	 
	mpu_fil_tmp[G_X] = (float)(FILT_TMP[G_X]) / (float)(FILTER_NUM_G);
	mpu_fil_tmp[G_Y] = (float)(FILT_TMP[G_Y]) / (float)(FILTER_NUM_G);
	mpu_fil_tmp[G_Z] = (float)(FILT_TMP[G_Z]) / (float)(FILTER_NUM_G);


	/*坐标转换*/
	Transform(mpu_fil_tmp[A_X], mpu_fil_tmp[A_Y], mpu_fil_tmp[A_Z], & Acc.x, & Acc.y, & Acc.z);
	Transform(mpu_fil_tmp[G_X], mpu_fil_tmp[G_Y], mpu_fil_tmp[G_Z], & Gyro.x, & Gyro.y, & Gyro.z);

	//	 Gyro_deg.x =  Gyro.x *TO_ANGLE;
	//	 Gyro_deg.y =  Gyro.y *TO_ANGLE;
	//	 Gyro_deg.z =  Gyro.z *TO_ANGLE;
	 Gyro_deg.x =  Gyro.x *MPU9250G_1000dps;
	 Gyro_deg.y =  Gyro.y *MPU9250G_1000dps;
	 Gyro_deg.z =  Gyro.z *MPU9250G_1000dps;
	//  MPU9250G_1000dps



}












void AHRS_S:: initial_data_filed()
{
 

		ahrs. mpu_data_ok = 1;
		MPU9250_OFF;

	 
}