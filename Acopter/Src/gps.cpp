#include "gps.h"
#include <math.h>
#include <algorithm>
#include "Butter.h"
#include "derivation.h"
#include "mymath.h"
#include "imu.h"
#include "time.h"
#include "gps_kalman_v3.h"

#include "include.h"

#include "imu_ekf2.h"

GPS gps;
//该函数需要修改，以适应其他国家
void GPS::GPS_pre(float dT)
{
  //static float px_temp;
  //static float py_temp;
  //static int i;
  if((gps.gps_data_ready_flag) )                 //gps接收数据完成，开始解算gps信息
  {
    float T=time.Get_Cycle_T(5);
	imu_ekf2.GPS_is_valid = 1;
    int i;
    GPSParse(gps.GpsBuffer);                                                //解算GPS信息
    gps_data_ready_flag=0;                                                  //清除标志位
    //判断定位质量是否满足条件
    //if(/*SatNum>4 &&*/ Status=='c' /*&& Pubx_Status[0]!='N'*/)
	if((SatNum>5) && (HDOP<30))
    {
      //满足条件，认为定位效果比较理想
      gps.gps_state_flag=1;
    }
    else
    {
      gps.gps_state_flag=0;
    }
    //
    if(gps.gps_state_flag==1)                                                   //对原始数据进行滤波
    {
      if(gps_sethome_flag==0)
      {
        gps_sethome(&gps.lat,&gps.lon,&gps.alt);//设定home点的 纬度 经度 海拔
      }
      if(gps_sethome_flag==1)
      {
        map_projection_project_haversine(gps.lat, gps.lon, px, py);
      }
      //N E D 速度信息 vn ve vd 单位：m/s
	  pz = (gps.alt - home_alt);
      for(i=4;i>0;i--)
      {
         py_temp[i]=py_temp[i-1];
         px_temp[i]=px_temp[i-1];
		 pz_temp[i]=pz_temp[i - 1];
      }
      py_temp[0]=py;
      px_temp[0]=px;
	  pz_temp[0]=pz;
	  
      ve=(py_temp[0]-py_temp[2])/(2.0*T);
      vn=(px_temp[0]-px_temp[2])/(2.0*T);
	  vd=(pz_temp[0]-pz_temp[2])/(2.0*T);
      COG_D=atan2(ve,vn);
      vd=vd;
      //经纬高偏差信息 单位：m
      StdDevLat=StdDevLat;
      StdDevLon=StdDevLon;
      StdDevAlt=StdDevAlt;
	  gps_pos_x_b = px*cos(imu_ekf2.yaw*M_PI_F / 180.0f) + py *sin(imu_ekf2.yaw*M_PI_F / 180.0f);
	  gps_pos_y_b = -px*sin(imu_ekf2.yaw*M_PI_F / 180.0f) + py *cos(imu_ekf2.yaw*M_PI_F / 180.0f);
	  
	  gps_v_x_b = gps.vn *cos(imu_ekf2.yaw*M_PI_F / 180.0f) + gps.ve *sin(imu_ekf2.yaw*M_PI_F / 180.0f);
	  gps_v_y_b = -gps.vn *sin(imu_ekf2.yaw*M_PI_F / 180.0f) + gps.ve *cos(imu_ekf2.yaw*M_PI_F / 180.0f);
       
	 //  gps_kalman_v3_update.Gps_kalman_v3_xy_update(T);
    }
  }
  else
  {
    //gps接收数据未完成
  }
}

int GPS::map_projection_project_haversine(double lat, double lon, float & x, float & y)
{
  if (gps_sethome_flag!=1) {
          return -1;
  }
  
  double lat_rad = lat * DEG_TO_RAD;
  double lon_rad = lon * DEG_TO_RAD;
  
  double home_lat_rad = home_lat * DEG_TO_RAD;
  double home_lon_rad = home_lon * DEG_TO_RAD;
  
  double sin_lat = sin(lat_rad);
  double cos_lat = cos(lat_rad);
  double cos_d_lon = cos(lon_rad - home_lon_rad);
  
  double d_lat_half = (lat_rad - home_lat_rad) * 0.5;
  double d_lon_half = (lon_rad - home_lon_rad) * 0.5;
  
  double sin_power_d_lat_half = sin(d_lat_half) * sin(d_lat_half);
  double sin_power_d_lon_half = sin(d_lon_half) * sin(d_lon_half);
  
  double home_sin_lat=sin(home_lat_rad);
  double home_cos_lat=cos(home_lat_rad);
  double home_sin_lon=sin(home_lon_rad);
  double home_cos_lon=cos(home_lon_rad);
  
  double arg = sqrt(sin_power_d_lat_half + home_cos_lat*cos_lat*sin_power_d_lon_half);
  
  if (arg > 1.0) {
          arg = 1.0;
  } else if (arg < -1.0) {
          arg = -1.0;
  }
  
  double c = 2*asin(arg);
  
  double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

  x = k * (home_cos_lat * sin_lat - home_sin_lat * cos_lat * cos_d_lon) * (CONSTANTS_RADIUS_OF_EARTH + alt);
  y = k * cos_lat * sin(lon_rad - home_lon_rad) * (CONSTANTS_RADIUS_OF_EARTH + alt);

  return 0;
}

int GPS::map_projection_project(double lat, double lon, float & x, float & y)
{
  if (gps_sethome_flag!=1) {
          return -1;
  }

  double lat_rad = lat * DEG_TO_RAD;
  double lon_rad = lon * DEG_TO_RAD;
  
  double home_lat_rad = home_lat * DEG_TO_RAD;
  double home_lon_rad = home_lon * DEG_TO_RAD;

  double sin_lat = sin(lat_rad);
  double cos_lat = cos(lat_rad);
  double cos_d_lon = cos(lon_rad - home_lon_rad);
  
  double home_sin_lat=sin(home_lat_rad);
  double home_cos_lat=cos(home_lat_rad);
  double home_sin_lon=sin(home_lon_rad);
  double home_cos_lon=cos(home_lon_rad);

  double arg = home_sin_lat * sin_lat + home_cos_lat * cos_lat * cos_d_lon;

  if (arg > 1.0) {
          arg = 1.0;

  } else if (arg < -1.0) {
          arg = -1.0;
  }

  double c = acos(arg);
  double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

  x = k * (home_cos_lat * sin_lat - home_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
  y = k * cos_lat * sin(lon_rad - home_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

  return 0;
}
int GPS::map_projection_project_test(double lat,double lon, float & x, float & y)
{
  if (gps_sethome_flag!=1) {
          return -1;
  }
  x = (lat - home_lat) * DEG_TO_RAD *  (CONSTANTS_RADIUS_OF_EARTH + alt);
  y = (lon - home_lon) * DEG_TO_RAD * (CONSTANTS_RADIUS_OF_EARTH + alt);
  return 0;
}

void GPS::gps_sethome(double * lat, double * lon, double * alt)
{
    const int N=100;
    static int index;
    home_lat+=*lat;
    home_lon+=*lon;
    home_alt+=*alt;
    if(index++==N)
    {
      home_lat=home_lat/(N+1.0);
      home_lon=home_lon/(N+1.0);
      home_alt=home_alt/(N+1.0);
      if((abs(home_lat - *lat)>3.0f) && (abs(home_lon - *lon)>3.0f))
      {
        index=0;
        gps_sethome_flag=0;
        home_lat=0.0f;
        home_lon=0.0f;
        home_alt=0.0f;
      }
      else
      {
        gps_sethome_flag=1;     //home点设定好后，设定标志位
        index=0;
      }
    }
}


uint16_t GPS::GpsFindStr(uint8_t *str,uint8_t *ptr)
{
  uint16_t index=0;
  uint8_t *STemp=NULL;
  uint8_t *PTemp=NULL;
  uint8_t *MTemp=NULL;
  if(0==str||0==ptr)
	return 0;
  for(STemp=str;*STemp!='\0';STemp++)	 //依次查找字符串
  {
    index++;  //当前偏移量加1
    MTemp=STemp; //指向当前字符串
    //比较
    for(PTemp=ptr;*PTemp!='\0';PTemp++)
    {	
      if(*PTemp!=*MTemp)
         break;
         MTemp++;
    }
    if(*PTemp=='\0')  //出现了所要查找的字符，退出
	break;
  }
  return index;
}

void GPS::GPSParse(uint8_t *GpsBuffer)
{
  static short CommaNum=0; //逗号数
  uint8_t BufIndex=0; //数字量
  static uint8_t Sbuf;
  uint8_t *Pstr;
  uint16_t index;
  //GNRMC
  //uint8_t GpsBuffertest[NMEA_COUNT_MAX]= "$GNGGA,072907.40,3101.65933,N,12126.26183,E,1,11,0.99,10.2,M,9.9,M,,*4F";//"$GNRMC,073121.40,A,3101.65878,N,12126.26145,E,0.045,,121016,,,A*6D";
#if 0
  index= GpsFindStr(GpsBuffer,(uint8_t*)"$GNRMC,");//查找
  if((index>=1)&&(index<=10))
  {
    CommaNum=0;
    Pstr=GpsBuffer+index+6;	 //找到GNRMC，后面的地址
    do
    {
      	Sbuf=*Pstr++;	
      	switch(Sbuf)
      	{
          case ',':CommaNum++;  //通过逗号的数目来进行状态分类
            BufIndex=0;
            break;
          default:
            switch (CommaNum)
            {
                case 0:
                {
                    gpsInfo.UtcTime[BufIndex] = Sbuf; 
                    if (*Pstr == ',')
                            gpsInfo.UtcTime[BufIndex + 1] = '\0';
                    break;
                }
                case 1:
                {
                    gpsInfo.Status = Sbuf;
                    break;
                }
                case 2:
                {
                    gpsInfo.Lat[BufIndex] = Sbuf;
                    if (*Pstr == ',')
                      gpsInfo.Lat[BufIndex + 1] = '\0';
                    break;
                }
                case 3:
                {
                    gpsInfo.Northing_Indicator = Sbuf;
                    break;
                }
                case 4:
                {
                    gpsInfo.Lon[BufIndex] = Sbuf;
                    if (*Pstr == ',')
                      gpsInfo.Lon[BufIndex + 1] = '\0';
                    break;
                }
                case 5:
                {
                    gpsInfo.Easting_Indicator = Sbuf;
                    break;
                }
                case 6:
                {
                    gpsInfo.SOG[BufIndex] = Sbuf;
                    if (*Pstr == ',')
                      gpsInfo.SOG[BufIndex + 1] = '\0';
                    break;
                }
                case 7:
                {
                    break;
                }
                case 8:
                {
                    gpsInfo.Date[BufIndex] = Sbuf;
                    if (*Pstr == ',')
                      gpsInfo.Date[BufIndex + 1] = '\0';
                    break;
                }
                case 11:
                {
                    gpsInfo.Mode_Indicator = *(Pstr-2);
                    break;
                }
//                case 12:
//                {
//                    gpsInfo.Navigational_Status = Sbuf;
//                }
                default:break;
            }
            BufIndex++;	//
            break;
          }
      }while(Sbuf!='*');//直到出现“*”退出
    //字符串转数字
    UtcTime=atof((char*)gpsInfo.UtcTime);
    Status=gpsInfo.Status;
	//秒转度
	double lat_temp = strtod((char*)gpsInfo.Lat, NULL)*0.01;
	double lon_temp = strtod((char*)gpsInfo.Lon, NULL)*0.01;
	lat = (double)((int)(lat_temp)) + 100*(lat_temp - (double)((int)(lat_temp))) / 60.0f;
	lon = (double)((int)(lon_temp)) + 100*(lon_temp - (double)((int)(lat_temp))) / 60.0f;

    Northing_Indicator=gpsInfo.Northing_Indicator;
    Easting_Indicator=gpsInfo.Easting_Indicator;
    SOG=atof((char*)gpsInfo.SOG);
    Mode_Indicator=gpsInfo.Mode_Indicator;
  }
#endif
  //uint8_t GpsBuffertest[NMEA_COUNT_MAX]= "$GNGGA,072907.40,3101.65933,N,12126.26183,E,1,11,0.99,10.2,M,9.9,M,,*4F";
  index = GpsFindStr(GpsBuffer, (uint8_t*)"$GPGGA,");
  CommaNum = 0;
  BufIndex = 0;
  Pstr = GpsBuffer + index + 6;	 //找到$GNGGA，后面的地址
  do
  {
	  Sbuf = *Pstr++;
	  switch (Sbuf)
	  {
	  case ',':CommaNum++;  //通过逗号的数目来进行状态分类
		  BufIndex = 0;
		  break;
	  default:
		  switch (CommaNum)
		  {
		  case 0:
		  {
			  gpsInfo.UtcTime[BufIndex] = Sbuf;
			  if (*Pstr == ',')
				  gpsInfo.UtcTime[BufIndex + 1] = '\0';
			  break;
		  }
		  case 1:
		  {
			  gpsInfo.Lat[BufIndex] = Sbuf;
			  if (*Pstr == ',')
				  gpsInfo.Lat[BufIndex + 1] = '\0';
			  break;
		  }
		  case 2:
		  {
			  gpsInfo.Northing_Indicator = Sbuf;
			  break;
		  }
		  case 3:
		  {
			  gpsInfo.Lon[BufIndex] = Sbuf;
			  if (*Pstr == ',')
				  gpsInfo.Lon[BufIndex + 1] = '\0';
			  break;
		  }
		  case 4:
		  {
			  gpsInfo.Easting_Indicator = Sbuf;
			  break;
		  }
		  case 5:
		  {
			  break;
		  }
		  case 6:
		  {
			  gpsInfo.SVs[BufIndex] = Sbuf;
			  if (*Pstr == ',')
				  gpsInfo.SVs[BufIndex + 1] = '\0';
			  break;
		  }
		  case 7:
		  {
			  gpsInfo.HDOP[BufIndex] = Sbuf;
			  if (*Pstr == ',')
				  gpsInfo.HDOP[BufIndex + 1] = '\0';
			  break;
		  }
		  case 8:
		  {
			  gpsInfo.Alt[BufIndex] = Sbuf;
			  if (*Pstr == ',')
				  gpsInfo.Alt[BufIndex + 1] = '\0';
			  break;
		  }
		  default:break;
		  }
		  BufIndex++;	//
		  break;
	  }
  } while (Sbuf != '*');//直到出现“*”退出
						//字符串转数字
  alt = atof((char*)gpsInfo.Alt);
  //字符串转数字
  UtcTime = atof((char*)gpsInfo.UtcTime);
  //秒转度
  double lat_temp = strtod((char*)gpsInfo.Lat, NULL)*0.01;
  double lon_temp = strtod((char*)gpsInfo.Lon, NULL)*0.01;
  lat = (double)((int)(lat_temp)) + 100 * (lat_temp - (double)((int)(lat_temp))) / 60.0f;
  lon = (double)((int)(lon_temp)) + 100 * (lon_temp - (double)((int)(lon_temp))) / 60.0f;
  HDOP = atof((char*)gpsInfo.HDOP);
  SatNum = atoi((char*)gpsInfo.SVs);           //卫星个数
  Northing_Indicator = gpsInfo.Northing_Indicator;
  Easting_Indicator = gpsInfo.Easting_Indicator;
}

void GPS::buffer_fil()
{
  u8 data;
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == SET)
  {
    data = huart3.Instance->DR;
    //HAL_UART_Receive_DMA(&huart3, &data, sizeof(data));
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    if (state == 0 && data == '$')
    {
            state = 1;
            GpsBuffer[0] = data;
    }
    else if (state == 1 && data == 'G')
    {
            state = 2;
            GpsBuffer[1] = data;
    }
    else if (state == 2 && data=='P')
    {
            state = 3;
            GpsBuffer[2] = data;
    }
    else if (state == 3 && data == 'G')
    {
            state = 4;
            GpsBuffer[3] = data;
    }
    else if (state == 4 && data == 'G')
    {
            state = 5;
            GpsBuffer[4] = data;
            _data_len = 14;//有55个逗号
            _data_cnt = 0;
    }
    else if (state == 5 && _data_len>=0)
    {
            _data_cnt++;
            GpsBuffer[4+_data_cnt] = data;
              
            if (data == ',')
            {
                    _data_len--;
                    //if (_data_len < 1 || _data_cnt>190)
                    if (_data_len == 0 )
                    {
                            GpsBuffer[4+_data_cnt+1] = '*';
                            gps_data_ready_flag = 1;
                            state = 0;
                    }
            }
    }
    else
            state = 0;

  }
	/*HAL_UART_Receive_DMA(&huart3, (uint8_t *) GpsBuffer, sizeof(GpsBuffer));
	 gps_data_ready_flag = 1;*/
}

void GPS::data_recieve_from_GPS_and_send_to_PC(void)
{
	u8 data;
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == SET)//stm32接收到GPS
	{
		data = huart3.Instance->DR;
		//HAL_UART_Receive_DMA(&huart3, &data, sizeof(data));
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		HAL_UART_Transmit_DMA(&huart2, &data, 1);
	}
}
void GPS::data_recieve_from_PC_and_send_to_GPS(void)
{
	u8 data;
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET)
	{
		data = huart2.Instance->DR;
		//HAL_UART_Receive_DMA(&huart3, &data, sizeof(data));
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		HAL_UART_Transmit_DMA(&huart3, &data, 1);
	}
}