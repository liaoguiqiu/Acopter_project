#include "delay.h"
#include "rc.h"
#include "mymath.h"
 
Acopter_RC rc;

const short MAX_CH[CH_NUM] = { 1930, 1930, 1930, 1930, 2070, 2070, 2070, 2070 };	//ҡ�����
const short MIN_CH[CH_NUM] = { 1109, 1109, 1109, 1109, 970, 970, 970, 970 };	//ҡ����С
const char CH_DIR[CH_NUM] = {1,1,0,0,0,0,0,0};  //ҡ�˷���
const short CH_in_Mapping[CH_NUM] = { 1, 0, 2, 3, 4, 5, 6, 7 };    //ͨ��ӳ��

void Acopter_RC:: CH_Mapping_Fun(u16 *in, u16 *Mapped_CH)
{
	u8 i;
	for (i = 0; i < CH_NUM; i++)
	{
		*(Mapped_CH + i) = *(in + CH_in_Mapping[i]);
	}
}


void Acopter_RC::RC_Duty(float T, u16 tmp16_CH[CH_NUM])
{
	u8 i;
	s16 CH_TMP[CH_NUM];
	static u16 Mapped_CH[CH_NUM];

	if (NS == 1)
	{
		CH_Mapping_Fun(tmp16_CH, Mapped_CH);
	}
	else if (NS == 2)
	{
		CH_Mapping_Fun(RX_CH, Mapped_CH);
	}

	for (i = 0; i < CH_NUM; i++)
	{
		if ((u16)Mapped_CH[i] >(MAX_CH[i] + 500) || (u16)Mapped_CH[i] < (MIN_CH[i] - 500))
		{
			CH_Error[i] = 1;
			CLR_CH_Error[i] = 0;
		}
		else
		{
			CLR_CH_Error[i]++;
			if (CLR_CH_Error[i] > 200)
			{
				CLR_CH_Error[i] = 2000;
				CH_Error[i] = 0;
			}
		}

		if (NS == 1 || NS == 2)
		{
			if (CH_Error[i]) //��ͨ�����ݴ���
			{

			}
			else
			{
				//CH_Max_Min_Record();
				CH_TMP[i] = (Mapped_CH[i]); //ӳ�俽�����ݣ���Լ 1000~2000

				if (MAX_CH[i] > MIN_CH[i])
				{
					if (!CH_DIR[i])
					{
						CH[i] = (short)LIMIT((s16)((CH_TMP[i] - MIN_CH[i]) / (float)(MAX_CH[i] - MIN_CH[i]) * 1000 - CH_OFFSET), -500, 500); //��һ�������+-500
					}
					else
					{
						CH[i] = -(short)LIMIT((s16)((CH_TMP[i] - MIN_CH[i]) / (float)(MAX_CH[i] - MIN_CH[i]) * 1000 - CH_OFFSET), -500, 500); //��һ�������+-500
					}
				}
				else
				{
					fly_ready = 0;
				}
			}
		}
		else //δ�ӽ��ջ������źţ�ң�عرջ�ʧ�źţ�
		{

		}
		//=================== filter ===================================
		//  ȫ�������CH_filter[],0�����1������2���ţ�3���� ��Χ��+-500	
		//=================== filter =================================== 		

		filter_A = 3.14f * 20 * T;

		if (ABS(CH_TMP[i] - CH_filter[i]) <100)
		{
			CH_filter[i] += filter_A *(CH[i] - CH_filter[i]);
		}
		else
		{
			CH_filter[i] += 0.5f *filter_A *(CH[i] - CH_filter[i]);
		}
		// 					CH_filter[i] = Fli_Tmp;
		CH_filter_D[i] = (CH_filter[i] - CH_filter_Old[i]);
		CH_filter_Old[i] = CH_filter[i];
		CH_Old[i] = CH[i];
	}
	//======================================================================
	Fly_Ready(T);		//�����ж�
	//======================================================================
	if (++NS_cnt>200)  // 400ms  δ���ź��ߡ�
	{
		NS_cnt = 0;
		NS = 0;
	}
}


void Acopter_RC:: Fly_Ready(float T)
{
	if (CH_filter[2] < -400)  							//����С��10%
	{
		if (fly_ready && ready_cnt != -1) //������ɣ������˳�������������
		{
			//ready_cnt += 1000 *T;
		}
#if(USE_TOE_IN_UNLOCK)		
		if (CH_filter[3] < -400)
		{
			if (CH_filter[1] > 400)
			{
				if (CH_filter[0] > 400)
				{
					if (ready_cnt != -1)				   //����������˳�������������
					{
						ready_cnt += 3 * 1000 * T;
					}
				}

			}

		}
#else
		if (CH_filter[3] < -400)					      //��������		
		{
			if (ready_cnt != -1 && fly_ready)	//�ж��Ѿ��˳����������������Ѿ�����
			{
				ready_cnt += 1000 * T;
			}
		}
		else if (CH_filter[3] > 400)      			//��������
		{
			if (ready_cnt != -1 && !fly_ready)	//�ж��Ѿ��˳����������������Ѿ�����
			{
				ready_cnt += 1000 * T;
			}
		}
#endif		
		else if (ready_cnt == -1)						//4ͨ��(CH[3])��λ
		{
			ready_cnt = 0;
		}
	}
	else
	{
		ready_cnt = 0;
	}


	if (ready_cnt > 1000) // 1000ms 
	{
		ready_cnt = -1;
		//fly_ready = ( fly_ready==1 ) ? 0 : 1 ;
		if (!fly_ready)
		{
			// LED_STATES_1();
			fly_ready = 1;
			//mpu6050.Gyro_CALIBRATE = 2;
			for (short i = 0; i<7; i++)
			{
				sum_temp[i] = 0;
			}



		}
		else
		{
			// LED_STATES_0();
			fly_ready = 0;
		}
	}

}

void Acopter_RC:: Feed_Rc_Dog(u8 ch_mode) //400ms�ڱ������һ��
{
	NS = ch_mode;
	NS_cnt = 0;
}