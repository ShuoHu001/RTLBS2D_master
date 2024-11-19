#ifndef RTLBS_POINT3D
#define RTLBS_POINT3D
#include "rtlbs.h"
#include "vector3d.h"
#include "managers/logmanager.h"
#include "utility/serializable.h"




class Point3D :public Serializable {

public:
	RtLbsType x, y, z;
	unsigned m_pId;					/** @brief	������IDֵ	*/

public:
	explicit Point3D();
	explicit Point3D(RtLbsType _x, RtLbsType _y, RtLbsType _z);
	explicit Point3D(RtLbsType _x, RtLbsType _y, RtLbsType _z, unsigned pid);
	Point3D(const Point3D& p);
	Point3D(const Vector3D& v);
	~Point3D();

public:
	Point3D operator + (const Vector3D& v) const;
	Point3D operator + (const Point3D& p) const;
	Point3D& operator += (const Vector3D& v);
	Point3D operator - (const Vector3D& v) const;
	Vector3D operator - (const Point3D& p) const;
	Point3D& operator -= (const Vector3D& v);
	Point3D operator * (RtLbsType f) const;
	Point3D& operator *= (RtLbsType f);
	Point3D operator / (RtLbsType f) const;
	Point3D& operator /=(RtLbsType f);
	Point3D& operator = (const Point3D& p);
	RtLbsType operator [] (unsigned id) const;
	RtLbsType& operator [] (unsigned id);
	bool operator == (const Point3D& p) const;
	bool operator != (const Point3D& p) const;
	bool operator < (const Point3D& p) const; //xyz�����Ƚϴ�С
	bool operator > (const Point3D& p) const;
	Point3D operator * (const Euler& e) const;					//��̬����ת
	Point3D& operator *= (const Euler& e);						//��̬����ת
	bool IsZero() const;									//�Ƿ���ԭ������
	Point3D& Rotate(double phi);							//��Z����ת
	Point3D& Rotate(double phi, double theta);				//��Z����ת����Y����ת
	Point3D& Rotate(const Vector3D& axis, double theta);	//�ƹ̶�����ת
	void Round(unsigned digit);									//���ȱ���ָ��λС��
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	std::string ToString() const; //��ȡ�ַ���
	size_t GetHash() const; //��ϣӳ��ֵ
};

//some global math operation
inline Point3D operator + (const Vector3D& v, const Point3D& p) {
	return p + v;
}

inline Point3D operator * (RtLbsType f, const Point3D& p) {
	return p * f;
}

inline RtLbsType SquaredDistance(const Point3D& p0, const Point3D& p1) {
	return (p0 - p1).SquaredLength();
}

inline RtLbsType Distance(const Point3D& p0, const Point3D& p1) {
	return (p0 - p1).Length();
}

inline Point3D Lerp(const Point3D& p1, const Point3D& p2, RtLbsType t) {
	if (t < 0 || t>1) {
		LOG_WARNING << "Incorrect range of t-values <" << t << ">" << ENDL;
	}
	RtLbsType x = p1.x + t * (p2.x - p1.y);
	RtLbsType y = p1.y + t * (p2.y - p1.y);
	RtLbsType z = p1.z + t * (p2.z - p1.z);
	return Point3D(x, y, z);
}

#endif
