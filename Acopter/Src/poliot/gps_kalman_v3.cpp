#include "gps_kalman_v3.h"
#include "position.h"
#include "mymath.h"
#include <math.h>

GPS_KALMAN_V3 gps_kalman_v3_update;
void GPS_KALMAN_V3::Gps_kalman_v3_xy_update(float T)
{
	float acc_x_diff, acc_y_diff;
        //对加速度数据进行滤波
        //Gps_kalman_v3_accx_filter(&position.wx_acc, &acc_x_af);
        //Gps_kalman_v3_accy_filter(&position.wy_acc, &acc_y_af);
        //acc_x_af = acc_x_af + (2*3.14f*T*5.0f) *(position.wx_acc - acc_x_af);
        //wx_acc += (1 / (1 + 1 / (10.0f *3.14f *T))) *((imu_dcm.a_x*ahrs.Acc.x + imu_dcm.a_y*ahrs.Acc.y + imu_dcm.a_z*ahrs.Acc.z)/8200*9800 - wx_acc);
  
	//x_kalman_update.kalman_updata(T, gps.gps_pos_x_b*1000.0f, gps.gps_v_x_b*1000.0f ,position.wx_acc);
	//y_kalman_update.kalman_updata(T, gps.gps_pos_y_b*1000.0f, gps.gps_v_y_b*1000.0f, position.wy_acc);

 
	//使用单纯位置加速模型
	x_kal_fill.pos_acc_kalman(T, gps.gps_pos_x_b*1000.0f, my_deathzoom(position.wx_acc_old[79], 50));
	y_kal_fill.pos_acc_kalman(T, gps.gps_pos_y_b*1000.0f, my_deathzoom(-position.wy_acc_old[79], 50));
	for (int i = 9; i > 0; i--)
	{
		vx_old[i] = vx_old[i - 1];
		vy_old[i] = vy_old[i - 1];

	}
	vx_old[0] = x_kal_fill.X(1, 0);
	vy_old[0] = y_kal_fill.X(1, 0);
	acc_x_diff = (vx_old[0] - vx_old[2]) / 2.0f / T;
	acc_y_diff = (vy_old[0] - vy_old[2]) / 2.0f / T;
	acc_x_feed_back += (1 / (1 + 1 / (1.0 * 3.14f *T)))*(acc_x_diff - acc_x_feed_back);
	acc_y_feed_back += (1 / (1 + 1 / (1.0 * 3.14f *T)))*(acc_y_diff - acc_y_feed_back);


}
void GPS_KALMAN_X_V3_UPDATE::kalman_init(float T, float pos,float acc)
{
 
  if(set_flag==0)
  {
    P0(0, 0)=100;
    P0(0, 1)=0;
    P0(1, 0)=0;
    P0(1, 1)=100;
    H(0, 0)=1;
    H(0, 1)=0;
    
    r=0.00001;
    q1=100000;
    q2=100000;
    
    set_flag=1;
  }
  U(0, 0) = my_deathzoom_2(acc, 150);
  Z(0, 0) = pos;
  
  F(0, 0)=1.0;
  F(0, 1)=T;
  F(1, 0)=0.0;
  F(1, 1)=1.0;
  
  B(0, 0)=T*T;
  B(1, 0)=T;
  
  //R(0, 0)=r/T;
  R(0, 0)=r;
  
  //Q(0, 0)=q*T*T*T/3.0;
  //Q(0, 1)=q*T*T/2.0;
  //Q(1, 0)=q*T*T/2.0;
  //Q(1, 1)=q*T;
  Q(0, 0)=q1;
  Q(1, 0)=q2;
  
}

void GPS_KALMAN_X_V3_UPDATE::kalman_updata(float T, float gps_pos, float gps_v, float acc)
{
  kalman_init(T,gps_pos,acc);
  //if(gps.gps_data_ready_flag)
  if(fabs(last_pos-gps_pos)>0.000001)
 //  if(1)
  {
    xn=F*Xkf+B*U;
    P1=F*P0*F.transpose()+Q;
    temp1=H*P1*H.transpose()+R;
    K=P1*H.transpose()*temp1.I();
    Xkf=xn+K*(Z-H*xn);
    P0=(EYE2-K*H)*P1;
    last_pos=gps_pos;
  }
  else
  {
    Xkf=F*Xkf+B*U;
    P0=F*P0*F.transpose()+Q;
    //互补滤波校正速度信息
    acc_mms2 = U(0,0) + acc_i;//9800 *T;
    //积分获得速度
    speed_0 += my_deathzoom((acc_mms2), 150) *T; //死区可调
    //向传感器sensor逼近
    speed_0 += (1 / (1 + 1 / (speed_kp *3.14f *T)))
            *(my_deathzoom(gps_v, 0) - speed_0);       
    //速度误差积分
    speed_i += speed_ki*T*(gps_v  - speed_0);
    speed_i = LIMIT(speed_i, -500, 500);
    speed_0 +=speed_i;
    //加速度误差比例校正
    acc_mms2 += (1 / (1 + 1 / (acc_kp *3.14f *T)))
            *((speed_0 - speed_old) / T - acc_mms2);
    //加速度误差积分
    acc_temp=(speed_0 - speed_old)/T;
    acc_i += acc_ki *T *((speed_0 - speed_old) / T - acc_mms2);
    acc_i = LIMIT(acc_i, -500, 500);
    speed_old = speed_0;
    Xkf(1,0)=speed_0;
  }
}

//滤波函数 fir:FS=100 Fpass=3 Fstop=5 
void GPS_KALMAN_V3::Gps_kalman_v3_accx_filter(float * in, float * out)
{
  const int count=51;
  const double cof[count] = {
-0.00104871662482, -0.00279495033236,-0.004727481963386,-0.006728497150671,
  -0.008651942066035, -0.01033003952156, -0.01158209208506, -0.01222511714573,
    -0.0120856901927, -0.01101224209357,-0.008886975760452,-0.005636545665607,
   -0.00124068435115,  0.00426193722291,  0.01077116933812,  0.01812651104402,
    0.02611201454879,  0.03446482041995,  0.04288692464186,  0.05105954644292,
    0.05865927188804,  0.06537500745974,  0.07092470144348,   0.0750707854883,
    0.07763335565081,  0.07850024762103,  0.07763335565081,   0.0750707854883,
    0.07092470144348,  0.06537500745974,  0.05865927188804,  0.05105954644292,
    0.04288692464186,  0.03446482041995,  0.02611201454879,  0.01812651104402,
    0.01077116933812,  0.00426193722291, -0.00124068435115,-0.005636545665607,
  -0.008886975760452, -0.01101224209357,  -0.0120856901927, -0.01222511714573,
   -0.01158209208506, -0.01033003952156,-0.008651942066035,-0.006728497150671,
  -0.004727481963386, -0.00279495033236, -0.00104871662482
  };
  //static double inbuf[count];
  //static int index,i;//index用于更新输入序列  i用于计算序列，j用于指示系数
  double  outcome=0.0f;
  //更新数据，新的数据进入，旧的数据向后移动
  for(index_x=count-1;index_x>0;index_x--)
  {
      inbuf_x[index_x]=inbuf_x[index_x-1];
  }
  inbuf_x[0]=*in;
  //与系数相乘得到滤波后的数据
  for(i_x=0;i_x<count;i_x++)
  {
      outcome+=inbuf_x[i_x]*cof[i_x];
  }
  *out=(float)outcome;
}

void GPS_KALMAN_V3::Gps_kalman_v3_accy_filter(float * in, float * out)
{
  const int count=51;
  const double cof[count] = {
-0.00104871662482, -0.00279495033236,-0.004727481963386,-0.006728497150671,
  -0.008651942066035, -0.01033003952156, -0.01158209208506, -0.01222511714573,
    -0.0120856901927, -0.01101224209357,-0.008886975760452,-0.005636545665607,
   -0.00124068435115,  0.00426193722291,  0.01077116933812,  0.01812651104402,
    0.02611201454879,  0.03446482041995,  0.04288692464186,  0.05105954644292,
    0.05865927188804,  0.06537500745974,  0.07092470144348,   0.0750707854883,
    0.07763335565081,  0.07850024762103,  0.07763335565081,   0.0750707854883,
    0.07092470144348,  0.06537500745974,  0.05865927188804,  0.05105954644292,
    0.04288692464186,  0.03446482041995,  0.02611201454879,  0.01812651104402,
    0.01077116933812,  0.00426193722291, -0.00124068435115,-0.005636545665607,
  -0.008886975760452, -0.01101224209357,  -0.0120856901927, -0.01222511714573,
   -0.01158209208506, -0.01033003952156,-0.008651942066035,-0.006728497150671,
  -0.004727481963386, -0.00279495033236, -0.00104871662482
  };
  //static double inbuf[count];
  //static int index,i;//index用于更新输入序列  i用于计算序列，j用于指示系数
  double  outcome=0.0f;
  //更新数据，新的数据进入，旧的数据向后移动
  for(index_y=count-1;index_y>0;index_y--)
  {
      inbuf_y[index_y]=inbuf_y[index_y-1];
  }
  inbuf_y[0]=*in;
  //与系数相乘得到滤波后的数据
  for(i_y=0;i_y<count;i_y++)
  {
      outcome+=inbuf_y[i_y]*cof[i_y];
  }
  *out=(float)outcome;
}
