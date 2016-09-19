#include "datatrans.h"
#include "ctrl.h"
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

Datatrans data_trans;


void Datatrans:: send_all_fly_data_to_ANO(void)
{
	send_15_data((s16 )1, (s16) 2, (s16) 3,
		(s16) 4, (s16) 5, (s16) 6,
		(s16) 7,( s16 )8, (s16) 9,
		(float) imu_dcm.Roll, (float) imu_dcm.Pitch, (float) imu_dcm.Yaw,
		(s32) 0, (u8) 0, (u8) 0);
}
void Datatrans:: acopter_Send_Data(u8 *dataToSend , u8 length)
{
  HAL_UART_Transmit_DMA(&huart2, dataToSend, length);
  //HAL_UART_Transmit(&huart2, dataToSend, length,101);
}



void Datatrans:: send_15_data(s16 a_x, s16 a_y, s16 a_z,
	s16 g_x, s16 g_y, s16 g_z,
	s16 m_x, s16 m_y, s16 m_z,
	float angle_rol, float angle_pit, float angle_yaw,
	s32 alt, u8 fly_model, u8 armed)
{
	static u8 send_all_data = 0;
	static u8 send_all_data_cnt = 0;
	static u8 _cnt = 0;
	if (send_all_data == 0)
	{
		_cnt = 0;
		vs16 _temp;

		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0x02;
		data_to_send[_cnt++] = 0;

		_temp = a_x;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = a_y;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = a_z;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		_temp = g_x;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = g_y;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = g_z;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		_temp = m_x;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = m_y;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = m_z;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		/////////////////////////////////////////
		_temp = 0;
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		data_to_send[3] = _cnt - 4;

		u8 sum = 0;
		for (u8 i = 0; i<_cnt; i++)
			sum += data_to_send[i];
		data_to_send[_cnt++] = sum;


		u8 _cnt2 = 0;
		u8 _cnt3 = 0;
		vs32 _temp2 = alt;
		_cnt3 = _cnt;
		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0xAA;
		data_to_send[_cnt++] = 0x01;
		_cnt2 = _cnt;
		data_to_send[_cnt++] = 0;

		_temp = (int)(angle_rol * 100);
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = (int)(angle_pit * 100);
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);
		_temp = (int)(angle_yaw * 100);
		data_to_send[_cnt++] = BYTE1(_temp);
		data_to_send[_cnt++] = BYTE0(_temp);

		data_to_send[_cnt++] = BYTE3(_temp2);
		data_to_send[_cnt++] = BYTE2(_temp2);
		data_to_send[_cnt++] = BYTE1(_temp2);
		data_to_send[_cnt++] = BYTE0(_temp2);

		data_to_send[_cnt++] = fly_model;

		data_to_send[_cnt++] = armed;

		data_to_send[_cnt2] = _cnt - _cnt2 - 1;

		sum = 0;
		for (u8 i = _cnt3; i<_cnt; i++)
			sum += data_to_send[i];
		data_to_send[_cnt++] = sum;


		send_all_data = 1;
		send_all_data_cnt = 0;
	}

	if (send_all_data == 1)
	{
		acopter_Send_Data(&data_to_send[send_all_data_cnt], (u8)1);
		send_all_data_cnt++;
		if (send_all_data_cnt>_cnt)
		{
			send_all_data = 0;
		}
	}



}
void Datatrans:: ANO_DT_Send_PID(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
	u8 _cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x10 + group - 1;
	data_to_send[_cnt++] = 0;


	_temp = (short)(p1_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p1_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p1_d * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p2_d * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_p * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_i * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (short)(p3_d * 1000);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;

	u8 sum = 0;
	for (u8 i = 0; i<_cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;

	acopter_Send_Data(data_to_send, _cnt);
}
void  Datatrans::data_trans_with_ano()
{
	
        /*---------------参数改变---------------*/
        if (get_data_from_ANO == 1)
	{
	
		ANO_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5);
                	get_data_from_ANO = 0;
		return;
	}

         /*---------------返回参数---------------*/
	if (send_pid_para)
	{
		if (send_pid_para == 1)
		{

			ANO_DT_Send_PID(1, ctrl_s.ctrl_1.PID[PIDROLL].kp, ctrl_s.ctrl_1.PID[PIDROLL].ki, ctrl_s.ctrl_1.PID[PIDROLL].kd,
				ctrl_s.ctrl_1.PID[PIDPITCH].kp, ctrl_s.ctrl_1.PID[PIDPITCH].ki, ctrl_s.ctrl_1.PID[PIDPITCH].kd,
				ctrl_s.ctrl_1.PID[PIDYAW].kp, ctrl_s.ctrl_1.PID[PIDYAW].ki, ctrl_s.ctrl_1.PID[PIDYAW].kd);
			send_pid_para++;
			return;
		}
		else if (send_pid_para == 200)
		{


			ANO_DT_Send_PID(2, ctrl_s.ctrl_2.PID[PIDROLL].kp, ctrl_s.ctrl_2.PID[PIDROLL].ki, ctrl_s.ctrl_2.PID[PIDROLL].kd,
				ctrl_s.ctrl_2.PID[PIDPITCH].kp, ctrl_s.ctrl_2.PID[PIDPITCH].ki, ctrl_s.ctrl_2.PID[PIDPITCH].kd,
				ctrl_s.ctrl_2.PID[PIDYAW].kp, ctrl_s.ctrl_2.PID[PIDYAW].ki, ctrl_s.ctrl_2.PID[PIDYAW].kd);
			send_pid_para++; 
			return;

		}
		else if (send_pid_para == 300)
		{


			ANO_DT_Send_PID(3, ctrl_s.ctrl_2.PID[PIDROLL].kp, ctrl_s.ctrl_2.PID[PIDROLL].ki, ctrl_s.ctrl_2.PID[PIDROLL].kd,
				ctrl_s.ctrl_2.PID[PIDPITCH].kp, ctrl_s.ctrl_2.PID[PIDPITCH].ki, ctrl_s.ctrl_2.PID[PIDPITCH].kd,
				ctrl_s.ctrl_2.PID[PIDYAW].kp, ctrl_s.ctrl_2.PID[PIDYAW].ki, ctrl_s.ctrl_2.PID[PIDYAW].kd);
			 send_pid_para++;
			return;

		}
		else if (send_pid_para == 400)
		{


			ANO_DT_Send_PID(4, ctrl_s.ctrl_2.PID[PIDROLL].kp, ctrl_s.ctrl_2.PID[PIDROLL].ki, ctrl_s.ctrl_2.PID[PIDROLL].kd,
				ctrl_s.ctrl_2.PID[PIDPITCH].kp, ctrl_s.ctrl_2.PID[PIDPITCH].ki, ctrl_s.ctrl_2.PID[PIDPITCH].kd,
				ctrl_s.ctrl_2.PID[PIDYAW].kp, ctrl_s.ctrl_2.PID[PIDYAW].ki, ctrl_s.ctrl_2.PID[PIDYAW].kd);
			 send_pid_para++;
			return;

		}
		else if (send_pid_para == 500)
		{


			ANO_DT_Send_PID(5, ctrl_s.ctrl_2.PID[PIDROLL].kp, ctrl_s.ctrl_2.PID[PIDROLL].ki, ctrl_s.ctrl_2.PID[PIDROLL].kd,
				ctrl_s.ctrl_2.PID[PIDPITCH].kp, ctrl_s.ctrl_2.PID[PIDPITCH].ki, ctrl_s.ctrl_2.PID[PIDPITCH].kd,
				ctrl_s.ctrl_2.PID[PIDYAW].kp, ctrl_s.ctrl_2.PID[PIDYAW].ki, ctrl_s.ctrl_2.PID[PIDYAW].kd);
			 send_pid_para++;
			return;

		}
		else if (send_pid_para == 600)
		{


			ANO_DT_Send_PID(6, ctrl_s.ctrl_2.PID[PIDROLL].kp, ctrl_s.ctrl_2.PID[PIDROLL].ki, ctrl_s.ctrl_2.PID[PIDROLL].kd,
				ctrl_s.ctrl_2.PID[PIDPITCH].kp, ctrl_s.ctrl_2.PID[PIDPITCH].ki, ctrl_s.ctrl_2.PID[PIDPITCH].kd,
				ctrl_s.ctrl_2.PID[PIDYAW].kp, ctrl_s.ctrl_2.PID[PIDYAW].ki, ctrl_s.ctrl_2.PID[PIDYAW].kd);
			send_pid_para = 0;
			return;

		}

		send_pid_para++;
		return;
	}
	 
	  /*---------------波形发送---------------*/
		send_all_fly_data_to_ANO();
	

}

void Datatrans::ANO_data_receive_prepara(void)
{
   u8 data;
   
  if(  __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET)
  {
    data = huart2.Instance->DR ;
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    
    if (state == 0 && data == 0xAA)
  {
	  state = 1;
	  RxBuffer[0] = data;
  }
  else if (state == 1 && data == 0xAF)
  {
	  state = 2;
	  RxBuffer[1] = data;
  }
  else if (state == 2 && data<0XF1)
	  //else if(state==2&&(data==0X11))
  {
	  state = 3;
	  RxBuffer[2] = data;
  }
  else if (state == 3 && data<50)
  {
	  state = 4;
	  RxBuffer[3] = data;
	  _data_len = data;
	  _data_cnt = 0;
  }
  else if (state == 4 && _data_len>0)
  {
	  _data_len--;
	  RxBuffer[4 + _data_cnt++] = data;
	  if (_data_len == 0)
		  state = 5;
  }
  else if (state == 5)
  {
	  state = 0;
	  RxBuffer[4 + _data_cnt] = data;
	  get_data_from_ANO = 1;
  }
  else
	  state = 0;
    
    
  }
  
}

void Datatrans:: ANO_DT_Data_Receive_Anl(u8 *data_buf, u8 num)
{
	u8 sum = 0;
	for (u8 i = 0; i<(num - 1); i++)
		sum += *(data_buf + i);
	if (!(sum == *(data_buf + num - 1)))		return;		//判断sum
	if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))		return;		//判断帧头

	//返回参数
	if (*(data_buf + 2) == 0X02)
	{
		if (*(data_buf + 4) == 0X01)
		{
			send_pid_para = 1;
			 
		}
	 
	}



	if (*(data_buf + 2) == 0X10)								//PID1
	{
		 

		 
		      ctrl_s.ctrl_1.PID[PIDROLL].kp  = 0.001*( (short)(*(data_buf+4)<<8)|*(data_buf+5) );
			  ctrl_s.ctrl_1.PID[PIDROLL].ki = 0.001*((short)(*(data_buf + 6) << 8) | *(data_buf + 7));
			  ctrl_s.ctrl_1.PID[PIDROLL].kd = 0.001*((short)(*(data_buf + 8) << 8) | *(data_buf + 9));
		     
			  ctrl_s.ctrl_1.PID[PIDPITCH].kp = 0.001*((short)(*(data_buf + 10) << 8) | *(data_buf + 11));
			  ctrl_s.ctrl_1.PID[PIDPITCH].ki = 0.001*((short)(*(data_buf + 12) << 8) | *(data_buf + 13));
			  ctrl_s.ctrl_1.PID[PIDPITCH].kd = 0.001*((short)(*(data_buf + 14) << 8) | *(data_buf + 15));
		   

	 
		      
			  ctrl_s.ctrl_1.PID[PIDYAW].kp = 0.001*((short)(*(data_buf + 16) << 8) | *(data_buf + 17));
			  ctrl_s.ctrl_1.PID[PIDYAW].ki = 0.001*((short)(*(data_buf + 18) << 8) | *(data_buf + 19));
			  ctrl_s.ctrl_1.PID[PIDYAW].kd = 0.001*((short)(*(data_buf + 20) << 8) | *(data_buf + 21));
		 

		ANO_DT_Send_Check(*(data_buf + 2), sum);
		//Param_SavePID();
		//Ctrl_Para_Init();
		//	flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X11)								//PID2
	{
		ctrl_s.ctrl_2.PID[PIDROLL].kp = 0.001*((short)(*(data_buf + 4) << 8) | *(data_buf + 5));
		ctrl_s.ctrl_2.PID[PIDROLL].ki = 0.001*((short)(*(data_buf + 6) << 8) | *(data_buf + 7));
		ctrl_s.ctrl_2.PID[PIDROLL].kd = 0.001*((short)(*(data_buf + 8) << 8) | *(data_buf + 9));
		ctrl_s.ctrl_2.PID[PIDPITCH].kp = 0.001*((short)(*(data_buf + 10) << 8) | *(data_buf + 11));
		ctrl_s.ctrl_2.PID[PIDPITCH].ki = 0.001*((short)(*(data_buf + 12) << 8) | *(data_buf + 13));
		ctrl_s.ctrl_2.PID[PIDPITCH].kd = 0.001*((short)(*(data_buf + 14) << 8) | *(data_buf + 15));
		ctrl_s.ctrl_2.PID[PIDYAW].kp = 0.001*((short)(*(data_buf + 16) << 8) | *(data_buf + 17));
		ctrl_s.ctrl_2.PID[PIDYAW].ki = 0.001*((short)(*(data_buf + 18) << 8) | *(data_buf + 19));
		ctrl_s.ctrl_2.PID[PIDYAW].kd = 0.001*((short)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
		//Ctrl_Para_Init();
		//flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X12)								//PID3
	{
		 	ANO_DT_Send_Check(*(data_buf + 2), sum);
		//PID_Para_Init();
		//flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X13)								//PID4
	{
		 	ANO_DT_Send_Check(*(data_buf + 2), sum);
		//PID_Para_Init();
		//flash_save_en_cnt = 1;
	}
	if (*(data_buf + 2) == 0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	if (*(data_buf + 2) == 0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}


}

 void Datatrans:: ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0] = 0xAA;
	data_to_send[1] = 0xAA;
	data_to_send[2] = 0xEF;
	data_to_send[3] = 2;
	data_to_send[4] = head;
	data_to_send[5] = check_sum;


	u8 sum = 0;
	for (u8 i = 0; i<6; i++)
		sum += data_to_send[i];
	data_to_send[6] = sum;

	acopter_Send_Data(data_to_send, 7);
}