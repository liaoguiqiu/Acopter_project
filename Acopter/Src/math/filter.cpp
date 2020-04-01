//#include "include.h"
#include "filter.h"
#include "mymath.h"
Myfilt filter;
// #define WIDTH_NUM 101
// #define FIL_ITEM  10

void Myfilt:: Moving_Average(float in, float moavarray[], u16 len, u16 fil_cnt[2], float *out)
{
	u16 width_num;

	width_num = len;

	if (++fil_cnt[0] > width_num)
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num) ? 0 : (fil_cnt[0] + 1);
	}

	moavarray[fil_cnt[0]] = in;
	*out += (in - (moavarray[fil_cnt[1]])) / (float)(width_num);

}

 


float Myfilt::Moving_Median(u8 item, u8 width_num, float in)
{
	u8 i, j;
	float t;
	float tmp[MED_WIDTH_NUM];

	if (item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM)
	{
		return 0;
	}
	else
	{
		if (++med_fil_cnt[item] >= width_num)
		{
			med_fil_cnt[item] = 0;
		}

		med_filter_tmp[item][med_fil_cnt[item]] = in;

		for (i = 0; i<width_num; i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}

		for (i = 0; i<width_num - 1; i++)
		{
			for (j = 0; j<(width_num - 1 - i); j++)
			{
				if (tmp[j] > tmp[j + 1])
				{
					t = tmp[j];
					tmp[j] = tmp[j + 1];
					tmp[j + 1] = t;
				}
			}
		}


		return (tmp[(u16)width_num / 2]);
	}
}

float Myfilt::Moving_Median_ae(u8 item, u8 width_num, float in)
{
	u8 i, j;
	float t;
	float tmp[MED_WIDTH_NUM];

	if (item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM)
	{
		return 0;
	}
	else
	{
		if (++med_fil_cnt[item] >= width_num)
		{
			med_fil_cnt[item] = 0;
		}

		med_filter_tmp[item][med_fil_cnt[item]] = in;

		for (i = 0; i<width_num; i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}

		for (i = 0; i<width_num - 1; i++)
		{
			for (j = 0; j<(width_num - 1 - i); j++)
			{
				if (tmp[j] > tmp[j + 1])
				{
					t = tmp[j];
					tmp[j] = tmp[j + 1];
					tmp[j + 1] = t;
				}
			}
		}
		float ae_temp = 0;

		for (i = 1; i<(width_num - 1); i++)
		{
			ae_temp += tmp[i];
		}

		return (ae_temp / (width_num - 2));
	}




}




void Myfilt::simple_3d_trans(Vector3f *ref, Vector3f*in, Vector3f *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x, h_tmp_y;

	h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
	h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));

	pn = ref->z < 0 ? -1 : 1;

	out->x = (h_tmp_x *in->x - pn *ref->x *in->z);
	out->y = (pn *h_tmp_y *in->y - ref->y *in->z);

	// 	 out->x = h_tmp_x *in->x - ref->x *in->z;
	// 	 out->y = ref->z *in->y - ref->y *in->z;

	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z;

}
void Myfilt::IIR_1st_lf(float T, float * in, float * out, float alfa)
{
	*out+= (1 / (1 + 1 / (alfa *3.14f *T))) *(*in - *out);
}

void Myfilt::IIR_second_lf(float T, float * in, float * save1, float * out, float alfa, float beta)
{
	IIR_1st_lf(T, in, save1, alfa);
	IIR_1st_lf(T, save1, out, beta);
}