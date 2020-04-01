#include "gps_kalman.h"
//#include "time.h"  
#include "gps.h"
#include "position.h"
#include "arm_math.h"

GPS_Kalman gps_kalman;

void GPS_Kalman::init_R_H_Q(void)
{
  
  R_srcRows = 6;
  R_srcColumns = 6;
  R_f32[0] = 10000.0;
  R_f32[7] = 50.0;
  R_f32[14] = 70.0;
  R_f32[21] = 10000.0;
  R_f32[28] = 50.0;
  R_f32[35] = 70.0;
  arm_mat_init_f32(&R, R_srcRows, R_srcColumns, (float32_t *)R_f32);
  
  
  H_srcRows = 6;
  H_srcColumns = 6;
  H_f32[0] = 1.0;
  H_f32[7] = 1.0;
  H_f32[14] = 1.0;
  H_f32[21] = 1.0;
  H_f32[28] = 1.0;
  H_f32[35] = 1.0;
  arm_mat_init_f32(&H, H_srcRows, H_srcColumns, (float32_t *)H_f32);
  
  
  Q_srcRows = 6;
  Q_srcColumns = 6;
  Q_f32[0]  = 0.0001;
  Q_f32[7]  = 0.05;
  Q_f32[14] = 0.1;
  Q_f32[21] = 0.0001;
  Q_f32[28] = 0.05;
  Q_f32[35] = 0.1;
  arm_mat_init_f32(&Q, Q_srcRows, Q_srcColumns, (float32_t *)Q_f32);
  
  
  P0_srcRows = 6;
  P0_srcColumns = 6;
  P0_f32[0]=10000.0;
  P0_f32[7]=10000.0;
  P0_f32[14]=10000.0;
  P0_f32[21]=10000.0;
  P0_f32[28]=10000.0;
  P0_f32[35]=10000.0;
  arm_mat_init_f32(&P0, P0_srcRows, P0_srcColumns, (float32_t *)P0_f32);
  
  P1_srcRows = 6;
  P1_srcColumns = 6;
  arm_mat_init_f32(&P1, P1_srcRows, P1_srcColumns, (float32_t *)P1_f32);
  
  Xkfout_srcRows = 6;
  Xkfout_srcColumns = 1;
  arm_mat_init_f32(&Xkfout, Xkfout_srcRows, Xkfout_srcColumns, (float32_t *)Xkfout_f32);
  
  Xn_srcRows = 6;
  Xn_srcColumns = 1;
  arm_mat_init_f32(&Xn, Xn_srcRows, Xn_srcColumns, (float32_t *)Xn_f32);
  
  //
  K_srcRows=6; K_srcColumns=6;
  arm_mat_init_f32(&K, K_srcRows, K_srcColumns, (float32_t *)K_f32);
  
  //初始化F的转置矩阵Ft
  Ft_srcRows = 6;
  Ft_srcColumns = 6;
  arm_mat_init_f32(&Ft, Ft_srcRows, Ft_srcColumns, (float32_t *)Ft_f32);
  
  //temp1 
  temp1_srcRows = 6;
  temp1_srcColumns = 6;
  arm_mat_init_f32(&temp1, temp1_srcRows, temp1_srcColumns, (float32_t *)temp1_f32);
  
  //temp2 
  temp2_srcRows = 6;
  temp2_srcColumns = 6;
  arm_mat_init_f32(&temp2, temp2_srcRows, temp2_srcColumns, (float32_t *)temp2_f32);
  
  //temp3
  temp3_srcRows = 6;
  temp3_srcColumns = 6;
  arm_mat_init_f32(&temp3, temp3_srcRows, temp3_srcColumns, (float32_t *)temp3_f32);
  
  //temp4 
  temp4_srcRows = 6;
  temp4_srcColumns = 6;
  arm_mat_init_f32(&temp4, temp4_srcRows, temp4_srcColumns, (float32_t *)temp4_f32);
  
  //temp5 
  temp5_srcRows = 6;
  temp5_srcColumns = 6;
  arm_mat_init_f32(&temp5, temp5_srcRows, temp5_srcColumns, (float32_t *)temp5_f32);
  
  //temp6 
  temp6_srcRows = 6;
  temp6_srcColumns = 6;
  arm_mat_init_f32(&temp6, temp6_srcRows, temp6_srcColumns, (float32_t *)temp6_f32);
  
  //temp7 
  temp7_srcRows = 6;
  temp7_srcColumns = 6;
  arm_mat_init_f32(&temp7, temp7_srcRows, temp7_srcColumns, (float32_t *)temp7_f32);
  
  //temp8 
  temp8_srcRows = 6;
  temp8_srcColumns = 1;
  arm_mat_init_f32(&temp8, temp8_srcRows, temp8_srcColumns, (float32_t *)temp8_f32);
  
  //temp9 
  temp9_srcRows = 6;
  temp9_srcColumns = 1;
  arm_mat_init_f32(&temp9, temp9_srcRows, temp9_srcColumns, (float32_t *)temp9_f32);
  
  //temp10 
  temp10_srcRows = 6;
  temp10_srcColumns = 1;
  arm_mat_init_f32(&temp10, temp10_srcRows, temp10_srcColumns, (float32_t *)temp10_f32);
  
  //temp11 
  temp11_srcRows = 6;
  temp11_srcColumns = 6;
  arm_mat_init_f32(&temp11, temp11_srcRows, temp11_srcColumns, (float32_t *)temp11_f32);
  
  //temp12
  temp12_srcRows = 6;
  temp12_srcColumns = 6;
  arm_mat_init_f32(&temp12, temp12_srcRows, temp12_srcColumns, (float32_t *)temp12_f32);
}

void GPS_Kalman::gps_kalman_update(float detT)
{
 static int gps_kalman_start_flag=1;
 if(gps_kalman_start_flag)
 {
   init_R_H_Q();
   gps_kalman_start_flag=0;
 }
  //更新F矩阵
  uint32_t F_srcRows, F_srcColumns;
  F_srcRows = 6;
  F_srcColumns = 6;
  F_f32[0]= 1.0;        F_f32[1]= detT;  F_f32[2]= 0.5*detT*detT;
  F_f32[7]= 1.0;        F_f32[8]= detT;  F_f32[14]= 1.0;
  F_f32[21]= 1.0;        F_f32[22]= detT;  F_f32[23]= 0.5*detT*detT;
  F_f32[28]= 1.0;        F_f32[29]= detT;  F_f32[35]= 1.0;
  arm_mat_init_f32(&F, F_srcRows, F_srcColumns, F_f32);
  
  //更新Xkfin矩阵
  uint32_t Xkfin_srcRows, Xkfin_srcColumns;
  Xkfin_srcRows = 6;
  Xkfin_srcColumns = 1;
  
  Xkfin_f32[0]=gps.px*1000.0*cos(imu_dcm.Yaw*M_PI_F/180.0f)+gps.py*1000.0*sin(imu_dcm.Yaw*M_PI_F/180.0f);
  Xkfin_f32[1]=gps.vn*1000.0*cos(imu_dcm.Yaw*M_PI_F/180.0f)+gps.ve*1000.0*sin(imu_dcm.Yaw*M_PI_F/180.0f);
  Xkfin_f32[2]=position.wx_acc,
  Xkfin_f32[3]=-gps.px*1000.0*sin(imu_dcm.Yaw*M_PI_F/180.0f)+gps.py*1000.0*cos(imu_dcm.Yaw*M_PI_F/180.0f);
  Xkfin_f32[4]=-gps.vn*1000.0*sin(imu_dcm.Yaw*M_PI_F/180.0f)+gps.ve*1000.0*cos(imu_dcm.Yaw*M_PI_F/180.0f);
  Xkfin_f32[5]=-position.wy_acc;
  arm_mat_init_f32(&Xkfin, Xkfin_srcRows, Xkfin_srcColumns, (float32_t *)Xkfin_f32);

  //更新Xn
  arm_mat_mult_f32(&F, &Xkfout, &Xn);
  arm_mat_trans_f32(&F,&Ft);
  
  //更新P1
  arm_mat_mult_f32(&F, &P0, &temp1);
  arm_mat_mult_f32(&temp1, &Ft, &temp2);
  arm_mat_add_f32(&temp2, &Q, &P1);
  //更新K
  //arm_mat_mult_f32(&H, &P1, &temp3);
  //arm_mat_mult_f32(&temp3, &H, &temp4);
  //arm_mat_add_f32(&temp4, &R, &temp5);
  arm_mat_add_f32(&P1, &R, &temp5);
  status=arm_mat_inverse_f32(&temp5,&temp6);
  //arm_mat_mult_f32(&P1, &H, &temp7);
  //arm_mat_mult_f32(&temp7, &temp6, &K);
  arm_mat_mult_f32(&P1, &temp6, &K);
  //更新Xkfout
  //arm_mat_mult_f32(&H, &Xn, &temp8);
  arm_mat_sub_f32(&Xkfin,&Xn,&temp9);
  arm_mat_mult_f32(&K, &temp9, &temp10);
  arm_mat_add_f32(&Xn, &temp10, &Xkfout);
  //更新P0
  //arm_mat_mult_f32(&K, &H, &temp11);
  arm_mat_sub_f32(&H,&K,&temp12);
  arm_mat_mult_f32(&temp12, &P1, &P0);
  
  //机体坐标系下的数据：前 右 下
  xkf_px_b=(float)Xkfout.pData[0]; //机体
  xkf_vn_b=(float)Xkfout.pData[1];;//(float)Xkfout.pData[1];
  xkf_accx_b=(float)Xkfout.pData[2];
  xkf_py_b=(float)Xkfout.pData[3];
  xkf_ve_b=(float)Xkfout.pData[4];
  xkf_accy_b=(float)Xkfout.pData[5];
  
  //导航坐标系下的数据：北 东 地
  xkf_px_n=xkf_px_b*cos(imu_dcm.Yaw*M_PI_F/180.0f)-xkf_py_b*sin(imu_dcm.Yaw*M_PI_F/180.0f);
  xkf_vn_n=xkf_vn_b*cos(imu_dcm.Yaw*M_PI_F/180.0f)-xkf_ve_b*sin(imu_dcm.Yaw*M_PI_F/180.0f);
  xkf_accx_n=xkf_accx_b*cos(imu_dcm.Yaw*M_PI_F/180.0f)-xkf_accy_b*sin(imu_dcm.Yaw*M_PI_F/180.0f);
  xkf_py_n=xkf_px_b*sin(imu_dcm.Yaw*M_PI_F/180.0f)+xkf_py_b*cos(imu_dcm.Yaw*M_PI_F/180.0f);
  xkf_ve_n=xkf_vn_b*sin(imu_dcm.Yaw*M_PI_F/180.0f)+xkf_ve_b*cos(imu_dcm.Yaw*M_PI_F/180.0f);
  xkf_accy_n=xkf_accx_b*sin(imu_dcm.Yaw*M_PI_F/180.0f)+xkf_accy_b*cos(imu_dcm.Yaw*M_PI_F/180.0f);
}