#ifndef __MATRIX3_H
#define __MATRIX3_H


#include "include.h"
#include "vector3.h"
// Ԫ������ΪT��3x3����
template <typename T>
class Matrix3 {
public:

	//�����������
	Vector3<T>        a, b, c;

	//�չ��캯��
	Matrix3<T>() {
	}

	//��ֵ���캯��
	Matrix3<T>(const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0) : a(a0), b(b0), c(c0) {
	}

	//��ֵ���캯��
	Matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz) : a(ax, ay, az), b(bx, by, bz), c(cx, cy, cz) {
	}

	//�������ò�����
	void operator        () (const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0)
	{
		a = a0; b = b0; c = c0;
	}

	//�����Ƿ����
	bool operator        == (const Matrix3<T> &m)
	{
		return (a == m.a && b == m.b && c == m.c);
	}

	//�����Ƿ����
	bool operator        != (const Matrix3<T> &m)
	{
		return (a != m.a || b != m.b || c != m.c);
	}

	//ȡ��
	Matrix3<T> operator        - (void) const
	{
		return Matrix3<T>(-a, -b, -c);
	}

	//���
	Matrix3<T> operator        + (const Matrix3<T> &m) const
	{
		return Matrix3<T>(a + m.a, b + m.b, c + m.c);
	}
	Matrix3<T> &operator        += (const Matrix3<T> &m)
	{
		return *this = *this + m;
	}

	//���
	Matrix3<T> operator        - (const Matrix3<T> &m) const
	{
		return Matrix3<T>(a - m.a, b - m.b, c - m.c);
	}
	Matrix3<T> &operator        -= (const Matrix3<T> &m)
	{
		return *this = *this - m;
	}

	//��������
	Matrix3<T> operator        * (const T num) const
	{
		return Matrix3<T>(a*num, b*num, c*num);
	}
	Matrix3<T> &operator        *= (const T num)
	{
		return *this = *this * num;
	}
	Matrix3<T> operator        / (const T num) const
	{
		return Matrix3<T>(a / num, b / num, c / num);
	}
	Matrix3<T> &operator        /= (const T num)
	{
		return *this = *this / num;
	}

	// ����һ������
	Vector3<T> operator         *(const Vector3<T> &v) const;

	//�����ת�ó���һ������
	Vector3<T>                  mul_transpose(const Vector3<T> &v) const;

	//��ȡx������
	Vector3<T>                  colx(void) const
	{
		return Vector3f(a.x, b.x, c.x);
	}

	//��ȡy������
	Vector3<T>        coly(void) const
	{
		return Vector3f(a.y, b.y, c.y);
	}

	//��ȡz������
	Vector3<T>        colz(void) const
	{
		return Vector3f(a.z, b.z, c.z);
	}

	//z��������ֵ
	void        set_colz(const Vector3<T> v)
	{
		a.z = v.x;	b.z = v.y; c.z = v.z;
	}

	//����һ��3�׷������
	Matrix3<T> operator *(const Matrix3<T> &m) const;

	Matrix3<T> &operator        *=(const Matrix3<T> &m)
	{
		return *this = *this * m;
	}

	//����ת��
	Matrix3<T>          transposed(void) const;

	Matrix3<T>          transpose(void)
	{
		return *this = transposed();
	}

	//����Ԫ������
	void        zero(void);

	//�����Ϊ��λ��
	void        identity(void) {
		a.x = b.y = c.z = 1;
		a.y = a.z = 0;
		b.x = b.z = 0;
		c.x = c.y = 0;
	}

	//���Ԫ���Ƿ����쳣ֵ
	bool        is_nan(void)
	{
		return a.is_nan() || b.is_nan() || c.is_nan();
	}

	//ŷ����ת���Ҿ���
	void        from_euler(const Vector3<T> &euler);

	//���Ҿ���תŷ����
	void        to_euler(float *roll, float *pitch, float *yaw);

	//ʹ�ý��ٶ���������������һ����ת
	void        rotate(const Vector3<T> &g);

	//ʹ�ý��ٶ�����(��X��Y)����������һ����ת
	void        rotateXY(const Vector3<T> &g);
};

typedef Matrix3<int16_t>                Matrix3i;
typedef Matrix3<uint16_t>               Matrix3ui;
typedef Matrix3<int32_t>                Matrix3l;
typedef Matrix3<uint32_t>               Matrix3ul;
typedef Matrix3<float>                  Matrix3f;
typedef Matrix3<double>                 Matrix3d;

#endif // MATRIX3_H




