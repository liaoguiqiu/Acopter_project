#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

class Quaternion
{
public:
	float        q1, q2, q3, q4;

	// ���캯��
	// roll=0, pitch=0, yaw=0
	Quaternion() {
		q1 = 1; q2 = q3 = q4 = 0;
	}

	//��ֵ���캯��
	Quaternion(const float _q1, const float _q2, const float _q3, const float _q4) :
		q1(_q1), q2(_q2), q3(_q3), q4(_q4) {
	}

	//�������ò�����
	void operator        ()(const float _q1, const float _q2, const float _q3, const float _q4)
	{
		q1 = _q1; q2 = _q2; q3 = _q3; q4 = _q4;
	}

	//��Ԫ����һ��
	void normalize(void);

	// ���ظ���Ԫ���ĵ�Ч��ת����
	void rotation_matrix(Matrix3f &m);

	// ���ظ���Ԫ���ĵ�Ч��ת�����е���������
	void vector_gravity(Vector3f &v);

	// ��һ�������ӵ�������ϵת������������ϵ
	void earth_to_body(Vector3f &v);

	//һ�����������������Ԫ��
	void Runge_Kutta_1st(Vector3f &g, float deltaT);

	//ŷ����ת��Ԫ��
	void from_euler(float roll, float pitch, float yaw);

	//��Ԫ��תŷ����
	void to_euler(float *roll, float *pitch, float *yaw);
};
#endif // QUATERNION_H














