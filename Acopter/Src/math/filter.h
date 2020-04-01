#ifndef __FILTER_H
#define __FILTER_H

#include "include.h"
#include "parameter.h"
#include "vector3.h"



#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  17


class Myfilt
{
public:

	Myfilt()
	{
	
	}
	//float Moving_Average(u8 item,u8 width_num,float in);
	void Moving_Average(float in, float moavarray[], u16 len, u16 fil_cnt[2], float *out);


	float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM];
	float med_filter_out[MED_FIL_ITEM];
	u8 med_fil_cnt[MED_FIL_ITEM];
	float Moving_Median(u8 item, u8 width_num, float in);

	void simple_3d_trans(Vector3f *ref, Vector3f *in, Vector3f *out);
	float Moving_Median_ae(u8 item, u8 width_num, float in);
	void IIR_1st_lf(float T, float * in,  float * out, float alfa);
	void IIR_second_lf(float T, float * in, float * save1, float * out, float alfa, float beta);

private:

};
extern Myfilt filter;
 
#endif
