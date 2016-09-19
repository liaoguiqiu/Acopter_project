#ifndef __FILTER_H
#define __FILTER_H

#include "include.h"
#include "parameter.h"
#include "vector3.h"
//float Moving_Average(u8 item,u8 width_num,float in);
void Moving_Average(float in, float moavarray[], u16 len, u16 fil_cnt[2], float *out);
float Moving_Median(u8 item, u8 width_num, float in);
 
void simple_3d_trans(Vector3f *ref, Vector3f *in, Vector3f *out);
float Moving_Median_ae(u8 item, u8 width_num, float in);
#endif
