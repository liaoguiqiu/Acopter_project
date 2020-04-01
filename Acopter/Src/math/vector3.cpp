#include "mymath.h"
#include "vector3.h"
#include "rotation.h"
#include "math.h"

#define HALF_SQRT_2 0.70710678118654757f	//根号2的值

template <typename T>
void Vector3<T>::rotate(enum Rotation rotation)
{
	T tmp;
	switch (rotation) {
	case ROTATION_NONE:
	case ROTATION_MAX:
		return;
	case ROTATION_YAW_45: {
		tmp = HALF_SQRT_2*(x - y);
		y = HALF_SQRT_2*(x + y);
		x = tmp;
		return;
	}
	case ROTATION_YAW_90: {
		tmp = x; x = -y; y = tmp;
		return;
	}
	case ROTATION_YAW_135: {
		tmp = -HALF_SQRT_2*(x + y);
		y = HALF_SQRT_2*(x - y);
		x = tmp;
		return;
	}
	case ROTATION_YAW_180:
		x = -x; y = -y;
		return;
	case ROTATION_YAW_225: {
		tmp = HALF_SQRT_2*(y - x);
		y = -HALF_SQRT_2*(x + y);
		x = tmp;
		return;
	}
	case ROTATION_YAW_270: {
		tmp = x; x = y; y = -tmp;
		return;
	}
	case ROTATION_YAW_315: {
		tmp = HALF_SQRT_2*(x + y);
		y = HALF_SQRT_2*(y - x);
		x = tmp;
		return;
	}
	case ROTATION_ROLL_180: {
		y = -y; z = -z;
		return;
	}
	case ROTATION_ROLL_180_YAW_45: {
		tmp = HALF_SQRT_2*(x + y);
		y = HALF_SQRT_2*(x - y);
		x = tmp; z = -z;
		return;
	}
	case ROTATION_ROLL_180_YAW_90: {
		tmp = x; x = y; y = tmp; z = -z;
		return;
	}
	case ROTATION_ROLL_180_YAW_135: {
		tmp = HALF_SQRT_2*(y - x);
		y = HALF_SQRT_2*(y + x);
		x = tmp; z = -z;
		return;
	}
	case ROTATION_PITCH_180: {
		x = -x; z = -z;
		return;
	}
	case ROTATION_ROLL_180_YAW_225: {
		tmp = -HALF_SQRT_2*(x + y);
		y = HALF_SQRT_2*(y - x);
		x = tmp; z = -z;
		return;
	}
	case ROTATION_ROLL_180_YAW_270: {
		tmp = x; x = -y; y = -tmp; z = -z;
		return;
	}
	case ROTATION_ROLL_180_YAW_315: {
		tmp = HALF_SQRT_2*(x - y);
		y = -HALF_SQRT_2*(x + y);
		x = tmp; z = -z;
		return;
	}
	case ROTATION_ROLL_90: {
		tmp = z; z = y; y = -tmp;
		return;
	}
	case ROTATION_ROLL_90_YAW_45: {
		tmp = z; z = y; y = -tmp;
		tmp = HALF_SQRT_2*(x - y);
		y = HALF_SQRT_2*(x + y);
		x = tmp;
		return;
	}
	case ROTATION_ROLL_90_YAW_90: {
		tmp = z; z = y; y = -tmp;
		tmp = x; x = -y; y = tmp;
		return;
	}
	case ROTATION_ROLL_90_YAW_135: {
		tmp = z; z = y; y = -tmp;
		tmp = -HALF_SQRT_2*(x + y);
		y = HALF_SQRT_2*(x - y);
		x = tmp;
		return;
	}
	case ROTATION_ROLL_270: {
		tmp = z; z = -y; y = tmp;
		return;
	}
	case ROTATION_ROLL_270_YAW_45: {
		tmp = z; z = -y; y = tmp;
		tmp = HALF_SQRT_2*(x - y);
		y = HALF_SQRT_2*(x + y);
		x = tmp;
		return;
	}
	case ROTATION_ROLL_270_YAW_90: {
		tmp = z; z = -y; y = tmp;
		tmp = x; x = -y; y = tmp;
		return;
	}
	case ROTATION_ROLL_270_YAW_135: {
		tmp = z; z = -y; y = tmp;
		tmp = -HALF_SQRT_2*(x + y);
		y = HALF_SQRT_2*(x - y);
		x = tmp;
		return;
	}
	case ROTATION_PITCH_90: {
		tmp = z; z = -x; x = tmp;
		return;
	}
	case ROTATION_PITCH_270: {
		tmp = z; z = x; x = -tmp;
		return;
	}
	case ROTATION_PITCH_180_YAW_90: {
		z = -z;
		tmp = -x; x = -y; y = tmp;
		return;
	}
	case ROTATION_PITCH_180_YAW_270: {
		x = -x; z = -z;
		tmp = x; x = y; y = -tmp;
		return;
	}
	case ROTATION_ROLL_90_PITCH_90: {
		tmp = z; z = y; y = -tmp;
		tmp = z; z = -x; x = tmp;
		return;
	}
	case ROTATION_ROLL_180_PITCH_90: {
		y = -y; z = -z;
		tmp = z; z = -x; x = tmp;
		return;
	}
	case ROTATION_ROLL_270_PITCH_90: {
		tmp = z; z = -y; y = tmp;
		tmp = z; z = -x; x = tmp;
		return;
	}
	case ROTATION_ROLL_90_PITCH_180: {
		tmp = z; z = y; y = -tmp;
		x = -x; z = -z;
		return;
	}
	case ROTATION_ROLL_270_PITCH_180: {
		tmp = z; z = -y; y = tmp;
		x = -x; z = -z;
		return;
	}
	case ROTATION_ROLL_90_PITCH_270: {
		tmp = z; z = y; y = -tmp;
		tmp = z; z = x; x = -tmp;
		return;
	}
	case ROTATION_ROLL_180_PITCH_270: {
		y = -y; z = -z;
		tmp = z; z = x; x = -tmp;
		return;
	}
	case ROTATION_ROLL_270_PITCH_270: {
		tmp = z; z = -y; y = tmp;
		tmp = z; z = x; x = -tmp;
		return;
	}
	case ROTATION_ROLL_90_PITCH_180_YAW_90: {
		tmp = z; z = y; y = -tmp;
		x = -x; z = -z;
		tmp = x; x = -y; y = tmp;
		return;
	}
	case ROTATION_ROLL_90_YAW_270: {
		tmp = z; z = y; y = -tmp;
		tmp = x; x = y; y = -tmp;
		return;
	}
	}
}

//求向量长度
template <typename T>
float Vector3<T>::length(void) const
{
	return pythagorous3(x, y, z);
}

//计算两个向量之间的夹角
template <typename T>
float Vector3<T>::angle(const Vector3<T> &v2) const
{
	return acosf(((*this)*v2) / (this->length()*v2.length()));
}

//向量叉积
template <typename T>
Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
	Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
	return temp;
}

//向量点积
template <typename T>
T Vector3<T>::operator *(const Vector3<T> &v) const
{
	return x*v.x + y*v.y + z*v.z;
}

//与标量相乘
template <typename T>
Vector3<T> Vector3<T>::operator *(const T num) const
{
	return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
Vector3<T> &Vector3<T>::operator *=(const T num)
{
	x *= num; y *= num; z *= num;
	return *this;  
}

//与标量相除
template <typename T>
Vector3<T> &Vector3<T>::operator /=(const T num)
{
	x /= num; y /= num; z /= num;
	return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator /(const T num) const
{
	return Vector3<T>(x / num, y / num, z / num);
}

//向量取反
template <typename T>
Vector3<T> Vector3<T>::operator -(void) const
{
	return Vector3<T>(-x, -y, -z);
}

//向量相加
template <typename T>
Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
	return Vector3<T>(x + v.x, y + v.y, z + v.z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
	x += v.x; y += v.y; z += v.z;
	return *this;
}

//向量相减
template <typename T>
Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
	return Vector3<T>(x - v.x, y - v.y, z - v.z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
	x -= v.x; y -= v.y; z -= v.z;
	return *this;
}

//对比两个向量是否相等
template <typename T>
bool Vector3<T>::operator ==(const Vector3<T> &v) const
{
	return (x == v.x && y == v.y && z == v.z);
}

//对比两个向量是否不相等
template <typename T>
bool Vector3<T>::operator !=(const Vector3<T> &v) const
{
	return (x != v.x && y != v.y && z != v.z);
}

//矩阵乘以一个行向量，得到一个行向量
template <typename T>
Vector3<T> Vector3<T>::operator *(const Matrix3<T> &m) const
{
	return Vector3<T>(*this * m.colx(),
		*this * m.coly(),
		*this * m.colz());
}

// 列向量乘以一个行向量,得到3x3矩阵
template <typename T>
Matrix3<T> Vector3<T>::mul_rowcol(const Vector3<T> &v2) const
{
	const Vector3<T> v1 = *this;
	return Matrix3<T>(v1.x * v2.x, v1.x * v2.y, v1.x * v2.z,
		v1.y * v2.x, v1.y * v2.y, v1.y * v2.z,
		v1.z * v2.x, v1.z * v2.y, v1.z * v2.z);
}

template <typename T>
void Vector3<T>::get_rollpitch(Vector3<T> &angle)
{
	angle.x = degrees(atan2f(y, z));
	angle.y = degrees(atan2f(-x, sqrtf(y * y + z* z)));
}

template <typename T>
void Vector3<T>::get_yaw(Vector3<T> &angle)
{
	angle.z = degrees(atan2f(y, x));
}

template <typename T>
bool Vector3<T>::is_nan(void) const
{
	return (isnan((float)x) || isnan((float)y) || isnan((float)z));
}


//只给浮点型定义了
template void Vector3<float>::rotate(enum Rotation);
template float Vector3<float>::length(void) const;
template Vector3<float> Vector3<float>::operator %(const Vector3<float> &v) const;
template float Vector3<float>::operator *(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator *(const Matrix3<float> &m) const;

template Vector3<float> &Vector3<float>::operator *=(const float num);
template Vector3<float> &Vector3<float>::operator /=(const float num);
template Vector3<float> &Vector3<float>::operator -=(const Vector3<float> &v);
template Vector3<float> &Vector3<float>::operator +=(const Vector3<float> &v);
template Vector3<float> Vector3<float>::operator /(const float num) const;
template Vector3<float> Vector3<float>::operator *(const float num) const;
template Vector3<float> Vector3<float>::operator +(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(void) const;
template bool Vector3<float>::operator ==(const Vector3<float> &v) const;
template bool Vector3<float>::operator !=(const Vector3<float> &v) const;
template float Vector3<float>::angle(const Vector3<float> &v) const;
template void Vector3<float>::get_rollpitch(Vector3<float> &angle);
template void Vector3<float>::get_yaw(Vector3<float> &angle);
template bool Vector3<float>::is_nan(void) const;








