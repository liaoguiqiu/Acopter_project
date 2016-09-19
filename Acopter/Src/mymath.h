/***
*mymath.h
*
*
*
*Purpose:
*       This file defined the functions and variables used by users
*       to fast computation the result of trigonometric functions and
*       the square root.
****/
//#include "stm32f4xx.h"

#ifndef __MYMATH_H__
#define __MYMATH_H__
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "include.h"
#include "rotation.h"
#include "vector3.h"
#include "matrix3.h"
#include "quaternion.h"


#define REAL   float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII   3.14159f
#define MY_PPPIII_HALF   1.570796f



#define M_PI 3.141592653f
#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f


#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))



//float LIMIT(float x, float y, float z);

float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
float my_pow(float a);
float my_sqrt(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x, float zoom);
float my_deathzoom_2(float x, float zoom);
float To_180_degrees(float x);
float my_pow_2_curve(float in, float a, float max);



float safe_asin(float v);

//浮点数限幅
float constrain_float(float amt, float low, float high);

//16位整型数限幅
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);

//16位无符号整型数限幅
uint16_t constrain_uint16(uint16_t amt, uint16_t low, uint16_t high);

//32位整型数限幅
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);

//角度转弧度
float radians(float deg);

//弧度转角度
float degrees(float rad);

//求平方
float sq(float v);

//2维向量长度
float pythagorous2(float a, float b);

//3维向量长度
float pythagorous3(float a, float b, float c); 

//4维向量长度
float pythagorous4(float a, float b, float c, float d);


#endif

