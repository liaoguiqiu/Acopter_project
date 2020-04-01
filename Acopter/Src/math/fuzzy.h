#ifndef _FUZZY_PID_H_
#define _FUZZY_PID_H_

struct Fuzzy_PID

{
	Fuzzy_PID()
	{

		//偏差隶属度赋值
		int PFF_data[4] = { 0, 500, 800, 1000 };
		for (short i = 0; i < 4; i++)
		{
			PFF[i] = PFF_data[i];
		}
		//变化隶属度赋值
		int DFF_data[4] = { 0, 500, 800, 1000 };
		for (short i = 0; i < 4; i++)
		{
			DFF[i] = DFF_data[i];
		}
		//输出隶属度赋值
		int UFF_data[7] = { 83, 93, 103, 113, 123, 133, 143 };
		for (short i = 0; i < 7; i++)
		{
			UFF[i] = UFF_data[i];
		}
           //规则赋值
		int rule_data[7][7] = {
			//-3,-2,-1, 0, 1, 2, 3     // 误差
			{ 6, 6, 5, 1, 3, 3, 3, },   //   -3
			{ 6, 5, 4, 1, 3, 3, 3, },   //   -2
			{ 5, 4, 4, 1, 3, 3, 3, },   //   -1
			{ 3, 2, 1, 0, 1, 2, 3, },   //    0
			{ 3, 3, 3, 1, 4, 4, 5, },   //    1
			{ 3, 3, 3, 1, 4, 5, 6, },   //    2
			{ 3, 3, 3, 1, 5, 6, 6 } };  //    3
		for (short i = 0; i < 7; i++)
		{
			for (short j = 0; j < 7; j++)
			{
				rule[i][j] = rule_data[i][j];
			}
		}

	}

	int PFF[4]  ;
	//输入量D语言值特征点
	int DFF[4]  ;


	  int UFF[7];

	  int rule[7][7];
	int  fuzzy_paramameter_output(int P, int D);
};




#endif