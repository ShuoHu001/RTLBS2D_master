#ifndef RTLBS_POINT2D
#define RTLBS_POINT2D
#include "vector2d.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "utility/define.h"

class Point2D {

public:
	RtLbsType x, y;
	RtLbsType distance;

public:
	HOST_DEVICE_FUNC Point2D();
	HOST_DEVICE_FUNC Point2D(RtLbsType _x, RtLbsType _y);
	HOST_DEVICE_FUNC explicit Point2D(RtLbsType _x, RtLbsType _y, RtLbsType _distance);
	HOST_DEVICE_FUNC Point2D(const Point2D& p);
	HOST_DEVICE_FUNC Point2D(const Vector2D& v);
	HOST_DEVICE_FUNC ~Point2D();

public:
	HOST_DEVICE_FUNC Point2D operator + (const Vector2D& v) const;
	HOST_DEVICE_FUNC Point2D operator + (const Point2D& p) const;
	HOST_DEVICE_FUNC Point2D& operator += (const Vector2D& v);
	HOST_DEVICE_FUNC Point2D operator - (const Vector2D& v) const;
	HOST_DEVICE_FUNC Vector2D operator - (const Point2D& p) const;
	HOST_DEVICE_FUNC Point2D& operator -= (const Vector2D& v);
	HOST_DEVICE_FUNC Point2D operator * (RtLbsType f) const;
	HOST_DEVICE_FUNC Point2D& operator *= (RtLbsType f);
	HOST_DEVICE_FUNC Point2D operator / (RtLbsType f) const;
	HOST_DEVICE_FUNC Point2D& operator /= (RtLbsType f);
	HOST_DEVICE_FUNC Point2D& operator = (const Point2D& p);
	HOST_DEVICE_FUNC RtLbsType operator [](unsigned id) const;
	HOST_DEVICE_FUNC RtLbsType& operator [] (unsigned id);
	HOST_DEVICE_FUNC bool operator == (const Point2D& p) const;
	HOST_DEVICE_FUNC bool operator != (const Point2D& p) const;

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

//some global math operation
HOST_DEVICE_FUNC inline Point2D operator + (const Vector2D& v, const Point2D& p) {
	return p + v;
}

HOST_DEVICE_FUNC inline Point2D operator * (RtLbsType f, const Point2D& p) {
	return p * f;
}

HOST_DEVICE_FUNC inline RtLbsType SquaredDistance(const Point2D& p0, const Point2D& p1) {
	return (p0 - p1).SquaredLength();
}

HOST_DEVICE_FUNC inline RtLbsType Distance(const Point2D& p0, const Point2D& p1) {
	return (p0 - p1).Length();
}

HOST_DEVICE_FUNC inline Point2D Lerp(const Point2D& p1, const Point2D& p2, RtLbsType t){ //线性插值函数

	if (t < 0 || t>1) { //GPU不支持
#ifdef __CUDA_ARCH__
		printf("Point2D Lerp:Incorrect range of t values\n");
#else
		LOG_WARNING << "Incorrect range of t-values <" << t << ">" << ENDL;
#endif	
	}
	RtLbsType x = p1.x + t * (p2.x - p1.x);
	RtLbsType y = p1.y + t * (p2.y - p1.y);
	return Point2D(x, y);
}
#endif
