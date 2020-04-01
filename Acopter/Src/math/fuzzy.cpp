#include "fuzzy.h"

#define FMAX    1000       /*����ֵ������ֵ*/ 

   


int Fuzzy_PID::fuzzy_paramameter_output (int P, int D)   /*ģ����������*/
{
  int    U;           /*ƫ��,ƫ��΢���Լ����ֵ�ľ�ȷ��*/
  unsigned int   PF[2], DF[2], UF[4];   /*ƫ��,ƫ��΢���Լ����ֵ��������*/
  int    Pn, Dn, Un[4];
  long   temp1, temp2;
  
  /*�����ȵ�ȷ��*/
  /*����PD��ָ������ֵ�����Ч������*/
  if (P > -PFF[3] && P < PFF[3])
  {
    if (P <= -PFF[2])
    {
      Pn = -2;
      PF[0] = (int)(FMAX*((float)(-PFF[2] - P) / (PFF[3] - PFF[2])));
    }
    else if (P <= -PFF[1])
    {
      Pn = -1;
      PF[0] = (int)(FMAX*((float)(-PFF[1] - P) / (PFF[2] - PFF[1])));
    }
    else if (P <= PFF[0])
    {
      Pn = 0;
      PF[0] = (int)(FMAX*((float)(-PFF[0] - P) / (PFF[1] - PFF[0])));
    }
    else if (P <= PFF[1])
    {
      Pn = 1; PF[0] = (int)(FMAX*((float)(PFF[1] - P) / (PFF[1] - PFF[0])));
    }
    else if (P <= PFF[2])
    {
      Pn = 2; PF[0] = (int)(FMAX*((float)(PFF[2] - P) / (PFF[2] - PFF[1])));
    }
    else if (P <= PFF[3])
    {
      Pn = 3; PF[0] = (int)(FMAX*((float)(PFF[3] - P) / (PFF[3] - PFF[2])));
    }
  }
  else if (P <= -PFF[3])
  {
    Pn = -2;   PF[0] = FMAX;
  }
  else if (P >= PFF[3])
  {
    Pn = 3;   PF[0] = 0;
  }
  PF[1] = FMAX - PF[0];
  if (D > -DFF[3] && D < DFF[3])
  {
    if (D <= -DFF[2])
    {
      Dn = -2; DF[0] = (int)(FMAX*((float)(-DFF[2] - D) / (DFF[3] - DFF[2])));
    }
    else if (D <= -DFF[1])
    {
      Dn = -1;
      DF[0] = (int)(FMAX*((float)(-DFF[1] - D) / (DFF[2] - DFF[1])));
    }
    else if (D <= DFF[0])
    {
      Dn = 0;
      DF[0] = (int)(FMAX*((float)(-DFF[0] - D) / (DFF[1] - DFF[0])));
    }
    else if (D <= DFF[1])
    {
      Dn = 1;
      DF[0] = (int)(FMAX*((float)(DFF[1] - D) / (DFF[1] - DFF[0])));
    }
    else if (D <= DFF[2])
    {
      Dn = 2; DF[0] = (int)(FMAX*((float)(DFF[2] - D) / (DFF[2] - DFF[1])));
    }
    else if (D <= DFF[3])
    {
      Dn = 3; DF[0] = (int)(FMAX*((float)(DFF[3] - D) / (DFF[3] - DFF[2])));
    }
  }
  else if (D <= -DFF[3])
  {
    Dn = -2;
    DF[0] = FMAX;
  }
  else if (D >= DFF[3])
  {
    Dn = 3;
    DF[0] = 0;
  }
  DF[1] = FMAX - DF[0];
  /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
  /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
  /*һ�㶼���ĸ�������Ч*/
  Un[0] = rule[Pn - 1 + 3][Dn - 1 + 3];
  Un[1] = rule[Pn + 3][Dn - 1 + 3];
  Un[2] = rule[Pn - 1 + 3][Dn + 3];
  Un[3] = rule[Pn + 3][Dn + 3];
  if (PF[0] <= DF[0])
    UF[0] = PF[0];
  else
    UF[0] = DF[0];
  if (PF[1] <= DF[0])
    UF[1] = PF[1];
  else
    UF[1] = DF[0];
  if (PF[0] <= DF[1])
    UF[2] = PF[0];
  else
    UF[2] = DF[1];
  if (PF[1] <= DF[1])
    UF[3] = PF[1];
  else
    UF[3] = DF[1];
  /*ͬ���������������ֵ���*/
  
  if (Un[0] == Un[1])
  {
    if (UF[0] > UF[1])
      UF[1] = 0;
    else
      UF[0] = 0;
  }
  if (Un[0] == Un[2])
  {
    if (UF[0] > UF[2])
      UF[2] = 0;
    else
      UF[0] = 0;
  }
  if (Un[0] == Un[3])
  {
    if (UF[0] > UF[3])
      UF[3] = 0;
    else
      UF[0] = 0;
  }
  if (Un[1] == Un[2])
  {
    if (UF[1] > UF[2])
      UF[2] = 0;
    else
      UF[1] = 0;
  }
  if (Un[1] == Un[3])
  {
    if (UF[1] > UF[3])
      UF[3] = 0;
    else
      UF[1] = 0;
  }
  if (Un[2] == Un[3])
  {
    if (UF[2] > UF[3])
      UF[3] = 0;
    else
      UF[2] = 0;
  }
  
  /*���ķ���ģ��*/
  /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
  if (Un[0] >= 0)
    Un[0] = UFF[Un[0]];
  else
    Un[0] = -UFF[-Un[0]];
  if (Un[1] >= 0)
    Un[1] = UFF[Un[1]];
  else
    Un[1] = -UFF[-Un[1]];
  if (Un[2] >= 0)
    Un[2] = UFF[Un[2]];
  else
    Un[2] = -UFF[-Un[2]];
  if (Un[3] >= 0)
    Un[3] = UFF[Un[3]];
  else
    Un[3] = -UFF[-Un[3]];
  
  temp1 = UF[0] * Un[0] + UF[1] * Un[1] + UF[2] * Un[2] + UF[3] * Un[3];
  temp2 = UF[0] + UF[1] + UF[2] + UF[3];
  U = temp1 / temp2;
  return U;
}
