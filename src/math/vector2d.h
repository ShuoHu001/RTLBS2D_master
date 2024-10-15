/** 
* @filename:	vector2d.h 
* @time:		2023/03/18 0:30:04 
* @author:		Hu Shuo
* @Email:		15991797156@163.com
* @location:	China, Shaanxi
* @brief:		Based 2D Vector in RTLBS 
*/
#ifndef RTLBS_VECTOR2D
#define RTLBS_VECTOR2D

#include "managers/logmanager.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include <math.h>

class Point2D;


class Vector2D {
public:
	/** @brief	the x data in two dimension	*/
	RtLbsType x;
	/** @brief	the y data in two dimension	*/
	RtLbsType y;
	/** @brief	whether it's a normal or not, by default, it's false	*/
	bool m_bNormal;

public:
	HOST_DEVICE_FUNC explicit Vector2D(bool normal = false);
	HOST_DEVICE_FUNC explicit Vector2D(double radian, bool normal = false);
	HOST_DEVICE_FUNC explicit Vector2D(RtLbsType _x, RtLbsType _y, bool normal = false);
	HOST_DEVICE_FUNC Vector2D(const Vector2D& v);
	HOST_DEVICE_FUNC Vector2D(const Point2D& p);
	HOST_DEVICE_FUNC ~Vector2D() {}

public:
	HOST_DEVICE_FUNC Vector2D operator + (const Vector2D& v) const;
	HOST_DEVICE_FUNC Vector2D& operator += (const Vector2D& v);
	HOST_DEVICE_FUNC Vector2D operator - (const Vector2D& v) const;
	HOST_DEVICE_FUNC Vector2D& operator -= (const Vector2D& v);
	HOST_DEVICE_FUNC Vector2D operator * (RtLbsType f) const;
	HOST_DEVICE_FUNC Vector2D& operator *= (RtLbsType f);
	HOST_DEVICE_FUNC Vector2D operator / (RtLbsType f) const;
	HOST_DEVICE_FUNC Vector2D& operator /= (RtLbsType f);
	HOST_DEVICE_FUNC Vector2D& operator = (const Vector2D& v);
	HOST_DEVICE_FUNC RtLbsType operator * (const Vector2D& v) const;
	HOST_DEVICE_FUNC bool operator == (const Vector2D& v) const;
	HOST_DEVICE_FUNC bool operator != (const Vector2D& v) const;
	HOST_DEVICE_FUNC RtLbsType operator [] (unsigned id) const;
	HOST_DEVICE_FUNC RtLbsType& operator [] (unsigned id);
	HOST_DEVICE_FUNC Vector2D operator - () const;

public:
	HOST_DEVICE_FUNC RtLbsType MaxComponent();
	HOST_DEVICE_FUNC RtLbsType MinComponent();
	HOST_DEVICE_FUNC RtLbsType Length() const;
	HOST_DEVICE_FUNC RtLbsType SquaredLength() const;
	HOST_DEVICE_FUNC Vector2D& Normalize();
	HOST_DEVICE_FUNC Vector2D& Rotate(double theta);
	HOST_DEVICE_FUNC Vector2D& RotatebyCos(double& costheta);//给定costheta值进行旋转
	HOST_DEVICE_FUNC RtLbsType GetTheta() const;			//求解向量对应的角度（0-2PI）


public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

//全局函数

HOST_DEVICE_FUNC inline RtLbsType Dot(const Vector2D v0, const Vector2D v1) {
	return v0.x * v1.x + v0.y * v1.y;
}

HOST_DEVICE_FUNC inline RtLbsType AbsDot(const Vector2D v0, const Vector2D v1) {
	RtLbsType r = Dot(v0, v1);
	return (r < 0.0) ? -r : r;
}

HOST_DEVICE_FUNC inline RtLbsType Cross(const Vector2D v0, const Vector2D v1) {
	return v0.x * v1.y - v0.y * v1.x;
}

HOST_DEVICE_FUNC inline Vector2D Normalize(const Vector2D& v) {
	RtLbsType len = v.Length();
	if (len == 0.0) {
#ifdef __CUDA_ARCH__
		printf("Vector2D Normalize:Try to normalize a zero length vector.\n");
#else
		LOG_ERROR << "Vector2D Normalize:Try to normalize a zero length vector." << ENDL;
#endif
		return Vector2D();
	}
	Vector2D re = v / len;
	re.m_bNormal = true;
	return re;
}

HOST_DEVICE_FUNC inline Vector2D Rotate(const Vector2D& v, double theta) {
	// 计算旋转后的坐标
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	Vector2D newVector(v);
	newVector.x = static_cast<RtLbsType>(v.x * cosTheta - v.y * sinTheta);
	newVector.y = static_cast<RtLbsType>(v.x * sinTheta + v.y * cosTheta);
	return newVector;
}

//根据v0构造出新的坐标系
HOST_DEVICE_FUNC inline void CoordinateSystem(const Vector2D& v0, Vector2D& v1) {
	RtLbsType invLen = static_cast<RtLbsType>(1.0 / sqrt(v0.x * v0.x + v0.y * v0.y));
	v1 = Vector2D(-v0.y * invLen, v0.x * invLen);
}


#endif
