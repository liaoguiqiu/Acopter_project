#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "include.h"

typedef struct
{
	u8 send_version;
	u8 send_status;
	u8 send_senser;
	u8 send_senser2;
	u8 send_pid1;
	u8 send_pid2;
	u8 send_pid3;
	u8 send_pid4;
	u8 send_pid5;
	u8 send_pid6;
	u8 send_rcdata;
	u8 send_offset;
	u8 send_motopwm;
	u8 send_power;
	u8 send_user;
	u8 send_speed;
	u8 send_location;

}dt_flag_t;

class Datatrans
{
public:
	short send_user_data_on_or_15_data;
	short send_all_usr_data;
	short send_pid_para;
	char get_data_from_ANO;
	u8 _data_len, _data_cnt;
	u8 state;
	dt_flag_t f;
	u8 data_to_send[50];	//发送数据缓存
	u8 user_data_to_send[50];	//发送数据缓存
	u8 RxBuffer[50];
	Datatrans()
	{
		send_user_data_on_or_15_data = 1;
		send_all_usr_data = 0;
		_data_len = 0;
		_data_cnt = 0;
		state = 0;
		get_data_from_ANO = 0;
	}



	void send_15_data(s16 a_x, s16 a_y, s16 a_z,
		s16 g_x, s16 g_y, s16 g_z,
		s16 m_x, s16 m_y, s16 m_z,
		float angle_rol, float angle_pit, float angle_yaw,
		s32 alt, u8 fly_model, u8 armed);
	//发送用户数据
	void Send_User_Datas(s16 user1, s16 user2, s16 user3,
		s16 user4, s16 user5, s16 user6,
		s16 user7, s16 user8, s16 user9,
		s16 user10, s16 user11, s16 user12,
		s16 user13, s16 user14, s16 user15,
		s16 user16, s16 user17, s16 user18,
		s16 user19, s16 user20
		);



	void acopter_Send_Data(u8 *dataToSend, u8 length);
	void send_all_fly_data_to_ANO(void);
	void data_trans_with_ano();
	void ANO_data_receive_prepara(void);
	void ANO_DT_Data_Receive_Anl(u8 *data_buf, u8 num);
	void ANO_DT_Send_Check(u8 head, u8 check_sum);
	void ANO_DT_Send_PID(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d);
private:

};

extern Datatrans data_trans;



#endif

