#ifndef __VECTOR3_H
#define __VECTOR3_H

#include "stdint.h"
#include "math.h"

template <typename T>
class Matrix3;

template <typename T>
class Vector3
{
public:
	T	x, y, z;

	//���캯������ʼ������ֵΪ0
	Vector3<T>()
	{
		x = y = z = 0;
	}

	//��ֵ���캯��
	Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0)
	{
	}

	void operator ()(const T x0, const T y0, const T z0)
	{
		x = x0; y = y0; z = z0;
	}

	//������ת
	void rotate(enum Rotation rotation);

	//��ȡ������ƽ��
	T  length_squared() const
	{
		return (T)(*this * *this);
	}

	//��ȡ����������
	float length(void) const;

	//������һ��
	void normalize()
	{
		*this /= length();
	}

	//��������
	void zero()
	{
		x = y = z = 0;
	}

	//���ع�һ������
	Vector3<T> normalized() const
	{
		return *this / length();
	}

	//������������֮��ļн�
	float angle(const Vector3<T> &v2) const;

	void get_rollpitch(Vector3<T> &angle);

	void get_yaw(Vector3<T> &angle);

	// ����Ƿ���Ԫ�ص�ֵΪ NAN
	bool is_nan(void) const;

	/*************************���������********************************/
	//���
	T operator *(const Vector3<T> &v) const;

	//���
	Vector3<T> operator %(const Vector3<T> &v) const;

	//��������
	Vector3<T> operator *(const T num) const;
	Vector3<T> &operator *=(const T num);

	//��������
	Vector3<T> operator  /(const T num) const;
	Vector3<T> &operator /=(const T num);

	//ȡ��
	Vector3<T> operator -(void) const;

	//���
	Vector3<T> operator +(const Vector3<T> &v) const;
	Vector3<T> &operator +=(const Vector3<T> &v);

	//���
	Vector3<T> operator -(const Vector3<T> &v) const;
	Vector3<T> &operator -=(const Vector3<T> &v);

	//�Ա����������Ƿ����
	bool operator ==(const Vector3<T> &v) const;

	//�Ա����������Ƿ����
	bool operator !=(const Vector3<T> &v) const;

	//�������һ�����������õ�һ��������
	Vector3<T> operator *(const Matrix3<T> &m) const;

	// ����������һ��������,�õ�3x3����
	Matrix3<T> mul_rowcol(const Vector3<T> &v) const;
	/*******************************************************************/

};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;

#endif // VECTOR3_H

