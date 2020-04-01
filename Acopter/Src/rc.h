#ifndef _RC_H
#define	_RC_H

#include "include.h"
#define CH_OFFSET 500
#define USE_TOE_IN_UNLOCK 0 // 0：默认解锁方式，1：外八解锁方式

class Acopter_RC
{
public:
  
  
  s16 CH[CH_NUM];
  
  u8 height_ctrl_mode;
  u16 RX_CH[CH_NUM];
  
  
  float CH_Old[CH_NUM];
  float CH_filter[CH_NUM];
  float CH_filter_Old[CH_NUM];
  float CH_filter_D[CH_NUM];
  u8 NS, CH_Error[CH_NUM];
  u16 NS_cnt, CLR_CH_Error[CH_NUM];
  float thr;
  float filter_A;
  
  u8 fly_ready ;
  s16 ready_cnt ;
  s32 sum_temp[7];
  /*-------构造函数------------*/
  Acopter_RC()
  {
    fly_ready = 0;
    
  }
  
  
  
  //方法
  void mode_switch(void);
  void CH_Mapping_Fun(u16 *in, u16 *Mapped_CH);
  void Fly_Ready(float T);
  void RC_Duty(float, u16 *);
  void Feed_Rc_Dog(u8 ch_mode);
  void Mode(void);
private:
  
};
 
extern Acopter_RC rc;

#endif

