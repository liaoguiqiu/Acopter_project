
#ifndef GPS_KALMAN_H
#define GPS_KALMAN_H

#include "arm_math.h"
#include "include.h"
class GPS_Kalman
{
public:
  GPS_Kalman()
  {
     //构造函数
    //init_R_H_Q();
  }
  float xkf_px_b;
  float xkf_vn_b;
  float xkf_accx_b;
  float xkf_py_b;
  float xkf_ve_b;
  float xkf_accy_b;
  
  float xkf_px_n;
  float xkf_vn_n;
  float xkf_accx_n;
  float xkf_py_n;
  float xkf_ve_n;
  float xkf_accy_n;
  
  arm_status status;
  
  arm_matrix_instance_f32 R;      /* Matrix R Instance */
  uint32_t R_srcRows, R_srcColumns;
  arm_matrix_instance_f32 H;      /* Matrix H Instance */
  uint32_t H_srcRows, H_srcColumns;
  arm_matrix_instance_f32 Q;      /* Matrix Q Instance */
  uint32_t Q_srcRows, Q_srcColumns;
  arm_matrix_instance_f32 P0;      /* Matrix P0 Instance */
  uint32_t P0_srcRows, P0_srcColumns;
  arm_matrix_instance_f32 P1;      /* Matrix P1 Instance */
  uint32_t P1_srcRows, P1_srcColumns;
  arm_matrix_instance_f32 F;      /* Matrix F Instance */
  
  arm_matrix_instance_f32 Ft;      /* Matrix F Instance */
  uint32_t Ft_srcRows, Ft_srcColumns;
  arm_matrix_instance_f32 Xkfin;    /* Matrix Xkfin Instance */
  
  arm_matrix_instance_f32 Xn;    /* Matrix Xkfin Instance */
  uint32_t Xn_srcRows, Xn_srcColumns;
  arm_matrix_instance_f32 Xkfout;    /* Matrix Xkfout Instance */
  uint32_t Xkfout_srcRows, Xkfout_srcColumns;
  arm_matrix_instance_f32 K;    /* Matrix Xkfout Instance */
  uint32_t K_srcRows, K_srcColumns;
  
  //更新P1时用到的临时变量        
  arm_matrix_instance_f32 temp1;
  uint32_t temp1_srcRows, temp1_srcColumns;
  arm_matrix_instance_f32 temp2;
  uint32_t temp2_srcRows, temp2_srcColumns;
  //更新K时用到的临时变量   
  arm_matrix_instance_f32 temp3;
  uint32_t temp3_srcRows, temp3_srcColumns;
  arm_matrix_instance_f32 temp4;
  uint32_t temp4_srcRows, temp4_srcColumns;
  arm_matrix_instance_f32 temp5;
  uint32_t temp5_srcRows, temp5_srcColumns;
  arm_matrix_instance_f32 temp6;
  uint32_t temp6_srcRows, temp6_srcColumns;
  arm_matrix_instance_f32 temp7;
  uint32_t temp7_srcRows, temp7_srcColumns;
  //更新Xkfout时需要的临时矩阵
  arm_matrix_instance_f32 temp8;
  uint32_t temp8_srcRows, temp8_srcColumns;
  arm_matrix_instance_f32 temp9;
  uint32_t temp9_srcRows, temp9_srcColumns;
  arm_matrix_instance_f32 temp10;
  uint32_t temp10_srcRows, temp10_srcColumns;
  //更新P0时需要的临时矩阵
  arm_matrix_instance_f32 temp11;
  uint32_t temp11_srcRows, temp11_srcColumns;
  arm_matrix_instance_f32 temp12;
  uint32_t temp12_srcRows, temp12_srcColumns;
  
  float32_t R_f32[36];
  float32_t H_f32[36];
  float32_t Q_f32[36];
  
  float32_t P0_f32[36];//
  float32_t P1_f32[36];//
  float32_t Xkfout_f32[6];
  float32_t Xkfin_f32[6];
  float32_t Xn_f32[6];
  float32_t Ft_f32[36];
  float32_t F_f32[36];
  float32_t K_f32[36];
  
  float32_t temp1_f32[36];
  float32_t temp2_f32[36];
  float32_t temp3_f32[36];
  float32_t temp4_f32[36];
  float32_t temp5_f32[36];
  float32_t temp6_f32[36];
  float32_t temp7_f32[36];
  float32_t temp11_f32[36];
  float32_t temp12_f32[36];
    
  float32_t temp8_f32[6];
  float32_t temp9_f32[6];
  float32_t temp10_f32[6];

  
  void init_R_H_Q(void);
  void gps_kalman_update(float detT);
private:

};
 
extern GPS_Kalman gps_kalman;

#endif
