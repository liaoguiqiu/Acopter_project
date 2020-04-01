#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <string.h>
//#include "arm_math.h"
#define  CONFIG_SET_UP_GPS_ON 0
#define NMEA_COUNT_MAX 100
#ifndef PI 
#define PI 3.1415926535898
#endif

#define M_PI_F 3.14159265358979323846f

#define EARTHR 6371004
#define EARTH_a 6378137.0      //�볤�� ��λ��m
#define EARTH_b 6356752.3142      //����� ��λ��m
#define EARTH_e2 0.00669447819799329
#define deg2rad(x)      x*PI/180.0
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m) */
#  define DBL_EPSILON 2.2204460492503131e-16  /* 1E-9 */

typedef struct
{
    //GNRMC
    uint8_t UtcTime[15]; //ʱ��
    uint8_t Status;   //��λ״̬
    uint8_t Lat[15];	//γ��
    uint8_t Northing_Indicator; //N=north,S=south
    uint8_t Lon[15];	 //����	
    uint8_t Easting_Indicator;  //E=east,W=west
    uint8_t SOG[15];	     //��λ��knots,Speed Over Ground
    uint8_t COG_True[15];    //Course Over Ground(true)
    uint8_t Date[15];   //Universal time coordinated  ddmmyy
    uint8_t Mode_Indicator; //Mode Indicator A=Autonomous, D=Differential, R=Fixed RTK, F=Float RTK, E=Dead Reckoning, N=None
    uint8_t Navigational_Status;        //S=Safe C=Caution U=Unsafe V=Not valid
    //GNVTG
    uint8_t COG_Mag[15];    //Course Over Ground(Magnetic)
    //GNGST
    uint8_t StdDevLat[15];      //Standard deviation of latitude error
    uint8_t StdDevLon[15];      //Standard deviation of longitude error
    uint8_t StdDevAlt[15];      //Standard deviation of altitude error
    //GNZDA
    uint8_t Day[15];
    uint8_t Month[15];
    uint8_t Year[15];
    uint8_t Local_zone_hours[15];  //Local zone hours 00 Local zone hours, 00 to +-13 hrs
    uint8_t Local_zone_minutes[15];     //Local zone minutes 00 Local zone minutes, 00 to 59
    //PUBX
    uint8_t Alt[15];    //Altitude (Above ellipsoid)
    uint8_t Pubx_Status[15];    //NF=no fix
    uint8_t Horizontal_Accuracy[15];    
    uint8_t Vertical_Accuracy[15];    
    uint8_t VD[15];     //Velocity Down
    uint8_t HDOP[15];   //Horizontal Dillution of Preciion
    uint8_t VDOP[15];   //Vertical Dillution of Preciion    
    uint8_t TDOP[15];   //Time Dillution of Preciion
    uint8_t SVs[15];    //Number of SVs used for Navigation
    uint8_t DRStatus;   //Dead Reckon Status Flags
}GPSINFO;



class GPS
{
public:
GPS()
{
   //���캯��
    gps_data_ready_flag=0;      //0��GPS����δ׼���ã������Խ��н��㣻1��GPS���ݶ�ȡ��ϣ����Խ��н���
    gps_state_flag=0;           //0�� GPS���ݲ����ã�1��GPS���ݿ���
    gps_sethome_flag=0;
    gps_start_fil_flag=0;
    memset(GpsBuffer,0,sizeof(GpsBuffer));
 }
float gps_pos_x_b;
float gps_pos_y_b;
float gps_v_x_b;
float gps_v_y_b;
  uint8_t GpsBuffer[NMEA_COUNT_MAX];
  uint8_t  state;
  short _data_cnt;
  short _data_len;
  bool gps_data_ready_flag;             //�Ƿ��������յ�GPSԭʼ����    1: yes  0:no
  bool gps_state_flag;                  //���յ��������Ƿ��ܹ���λ�ı�־λ  1��yes  0:no
  bool gps_sethome_flag;                //�Ƿ���ȷȷ���ҵ�ı�־λ       1��yes  0:no
  bool gps_start_fil_flag;
  GPSINFO gpsInfo;
  float UtcTime;
  uint8_t Status;
  double lat;
  uint8_t Northing_Indicator;
  double lon;
  uint8_t Easting_Indicator;
  float SOG;
  float COG_T;
  float COG_T_rad;
  float COG_D; //λ��΢�ֺ���ٶ��������еõ��ĽǶ���Ϣ
  float velocity_mps;
  uint8_t Mode_Indicator;
  float COG_M;
  float StdDevLat;
  float StdDevLon;
  float StdDevAlt;
  double alt;
  uint8_t Pubx_Status[15];
  float Horizontal_Accuracy;
  float Vertical_Accuracy;
  float HDOP;
  float VDOP;
  float TDOP;
  int SatNum;           //���Ǹ���
  float vn,ve,vd;
  float vn_COG,ve_COG,vd_COG;
  double home_lat,home_lon,home_alt;

  //double home_lat_rad,home_lon_rad,home_alt_rad;
  double ptx,pty,ptz;
  float px_temp[5];
  float py_temp[5];
  float pz_temp[5];
  float px,py,pz;
  //float vx,vy,vz;
  
  void buffer_fil();
  void GPS_pre(float dT);
  uint16_t GpsFindStr(uint8_t *str,uint8_t *ptr);
  void GPSParse(uint8_t *GpsBuffer);
  double Str2doubleLat(uint8_t *Lat);
  double Str2doubleLon(uint8_t *Lon);
  float Str2floatVelocity(uint8_t *Velocity);
  double Str2floatAlt(uint8_t *Alt);
  int Str2intSat(uint8_t *Sat);
  
  //�趨home���  ��γ��
  void gps_sethome(double * lat, double * lon, double * alt);
  int map_projection_project(double lat, double lon, float & x, float & y);
  int map_projection_project_haversine(double lat, double lon, float & x, float & y);
  int map_projection_project_test(double lat,double lon, float & x, float & y);
  
  void Gps_g2e_home(double* lat, double* lon, double* alt, double* xe, double* ye, double* ze);
  void Gps_g2e_normal(double* lat, double* lon, double* alt, double* xe, double* ye, double* ze);
  void Gps_e2t(double* xe, double* ye, double* ze, double* xt, double* yt, double* zt);
  void det_e(void);
  void data_recieve_from_GPS_and_send_to_PC(void);
  void data_recieve_from_PC_and_send_to_GPS(void);
private:

};
 
extern GPS gps;

#endif
