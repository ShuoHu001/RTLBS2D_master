#ifndef RTLBS_VECTOR3D
#define RTLBS_VECTOR3D

#include "rtlbs.h"
#include "managers/logmanager.h"
#include "utility/serializable.h"
#include "euler.h"
#include "vector2d.h"
class Point3D;



class Vector3D:public Serializable {
public:
	RtLbsType x;
	RtLbsType y;
	RtLbsType z;
	bool m_isNormal;

public:
	explicit Vector3D(bool normal = false);
	explicit Vector3D(RtLbsType _x, RtLbsType _y, RtLbsType _z, bool normal = false);
	Vector3D(const Vector3D& v);
	Vector3D(const Point3D& p);
	~Vector3D(){}

public:
	Vector3D operator + (const Vector3D& v) const;
	Vector3D& operator += (const Vector3D& v);
	Vector3D operator - (const Vector3D& v) const;
	Vector3D& operator -= (const Vector3D& v);
	Vector3D operator * (RtLbsType f) const;
	Vector3D& operator *= (RtLbsType f);
	Vector3D operator / (RtLbsType f) const;
	Vector3D& operator /= (RtLbsType f);
	Vector3D& operator = (const Vector3D& v);
	RtLbsType operator * (const Vector3D& v) const;
	Vector3D operator * (const Euler& e) const;						//姿态角旋转
	Vector3D& operator *= (const Euler& e);							//姿态角旋转
	bool operator == (const Vector3D& v) const;
	bool operator != (const Vector3D& v) const;
	RtLbsType operator [] (unsigned id) const;
	RtLbsType& operator [] (unsigned id);
	Vector3D operator - () const;

public:
	bool IsZero() const;												//是否是零向量
	RtLbsType MaxComponent();
	RtLbsType MinComponent();
	int GetMaxId() const;
	int GetMinId() const;
	RtLbsType Length() const;
	RtLbsType LengthXY() const;
	RtLbsType SquaredLength() const;
	RtLbsType SquaredLengthXY() const;
	Vector3D& Normalize();
	RtLbsType Dot(const Vector3D& v) const;
	RtLbsType AbsDot(const Vector3D& v) const;
	Vector3D Cross(const Vector3D& v) const;
	Vector3D& Rotate(double phi);
	Vector3D& Rotate(double phi, double theta);
	Vector3D& Rotate(const Vector3D& axis, double theta);
	Vector3D& RotatebyCos(double costheta);
	Vector3D& RotatebyCos(double costheta, double cosphi);
	Vector3D& RotatebyCos(const Vector3D& axis, double costheta);
	RtLbsType Azimuth() const;											//计算方位角
	RtLbsType Elevation() const;										//计算俯仰角
	std::string ToString() const;
	size_t GetHash() const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

inline RtLbsType Dot(const Vector3D& v0, const Vector3D& v1) {
	return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

inline RtLbsType AbsDot(const Vector3D& v0, const Vector3D& v1) {
	RtLbsType r = Dot(v0, v1);
	return (r < 0.0) ? -r : r;
}

inline Vector3D Cross(const Vector3D& v0, const Vector3D& v1) {
	return Vector3D(v0.y * v1.z - v0.z * v1.y, v0.z * v1.x - v0.x * v1.z, v0.x * v1.y - v0.y * v1.x, v0.m_isNormal && v1.m_isNormal);
}

inline Vector3D Normalize(const Vector3D& v) {
	RtLbsType len = v.Length();
	if (len == 0.0) {
		LOG_ERROR << "Try to normalize a zero length vector." << ENDL;
		return Vector3D();
	}
	Vector3D re = v / len;
	re.m_isNormal = true;
	return re;
}

inline Vector2D NormalizeXY(const Vector3D& v) {
	Vector2D v2d(v.x, v.y);
	RtLbsType len = v2d.Length();
	if (len == 0.0) {
		LOG_ERROR << "Try to normalize a zero length vector." << ENDL;
		return Vector2D();
	}
	Vector2D re = v2d / len;
	return re;
}

inline Vector3D Rotate(const Vector3D& v, double theta) {
	double matrix[9] = {
		cos(theta), -sin(theta), 0.0,
		sin(theta), cos(theta), 0.0,
		0.0, 0.0, 1.0
	};
	RtLbsType vector[3] = { v.x, v.y, v.z };
	RtLbsType retval[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, matrix, 3, vector, 1, 0.0, retval, 1);
	return Vector3D(retval[0], retval[1], retval[2]);
}

inline Vector3D Rotate(const Vector3D& v, double theta, double phi) {
	// Define the rotation matrices
	double rotateTheta[9] = {
		cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1
	};

	double rotatePhi[9] = {
		cos(phi), 0, sin(phi),
		0, 1, 0,
		-sin(phi), 0, cos(phi)
	};

	double rotationMatrix[9];
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1.0, rotateTheta, 3, rotatePhi, 3, 0.0, rotationMatrix, 3);

	double vector[3] = { v.x, v.y, v.z };
	double retval[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retval, 1);

	return Vector3D(retval[0], retval[1], retval[2], v.m_isNormal);
}

inline Vector3D Rotate(const Vector3D& v, const Vector3D& axis, double theta) {
	double costheta = cos(theta);
	double sintheta = sin(theta);

	// Define the rotation matrix
	double rotationMatrix[9] = {
		costheta + axis.x * axis.x * (1 - costheta), axis.x * axis.y * (1 - costheta) - axis.z * sintheta, axis.x * axis.z * (1 - costheta) + axis.y * sintheta,
		axis.y * axis.x * (1 - costheta) + axis.z * sintheta, costheta + axis.y * axis.y * (1 - costheta), axis.y * axis.z * (1 - costheta) - axis.x * sintheta,
		axis.z * axis.x * (1 - costheta) - axis.y * sintheta, axis.z * axis.y * (1 - costheta) + axis.x * sintheta, costheta + axis.z * axis.z * (1 - costheta)
	};

	double vector[3] = { v.x, v.y, v.z };
	double retvl[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retvl, 1);

	return Vector3D(retvl[0], retvl[1], retvl[2], v.m_isNormal);
}

inline Vector3D RotatebyCos(const Vector3D& v, double costheta) {
	double sintheta = sqrt(1 - costheta * costheta);
	double matrix[9] = {
		costheta, -sintheta, 0.0,
		sintheta, costheta, 0.0,
		0.0, 0.0, 1.0
	};
	RtLbsType vector[3] = { v.x, v.y, v.z };
	RtLbsType retval[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, matrix, 3, vector, 1, 0.0, retval, 1);
	return Vector3D(retval[0], retval[1], retval[2]);
}

inline Vector3D RotatebyCos(const Vector3D& v, double costheta, double cosphi) {
	
	double sintheta = sqrt(1 - costheta * costheta);
	double sinphi = sqrt(1 - sintheta * sintheta);
	
	double rotateTheta[9] = {
		costheta, -sintheta, 0,
		sintheta, costheta, 0,
		0, 0, 1
	};

	double rotatePhi[9] = {
		cosphi, 0, sinphi,
		0, 1, 0,
		-sinphi, 0, cosphi
	};

	double rotationMatrix[9];
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1.0, rotateTheta, 3, rotatePhi, 3, 0.0, rotationMatrix, 3);

	double vector[3] = { v.x, v.y, v.z };
	double retval[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retval, 1);

	return Vector3D(retval[0], retval[1], retval[2], v.m_isNormal);
}

inline Vector3D RotatebyCos(const Vector3D& v, const Vector3D& axis, double costheta) {
	double sintheta = sqrt(1 - costheta * costheta);

	// Define the rotation matrix
	double rotationMatrix[9] = {
		costheta + axis.x * axis.x * (1 - costheta), axis.x * axis.y * (1 - costheta) - axis.z * sintheta, axis.x * axis.z * (1 - costheta) + axis.y * sintheta,
		axis.y * axis.x * (1 - costheta) + axis.z * sintheta, costheta + axis.y * axis.y * (1 - costheta), axis.y * axis.z * (1 - costheta) - axis.x * sintheta,
		axis.z * axis.x * (1 - costheta) - axis.y * sintheta, axis.z * axis.y * (1 - costheta) + axis.x * sintheta, costheta + axis.z * axis.z * (1 - costheta)
	};

	double vector[3] = { v.x, v.y, v.z };
	double retvl[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retvl, 1);

	return Vector3D(retvl[0], retvl[1], retvl[2], v.m_isNormal);
}

//计算矢量v与X轴正方向的方位角
inline RtLbsType Azimuth(const Vector3D& v) {
	if (v.y < 0)
		return atan2(v.y, v.x) + TWO_PI;
	return atan2(v.y, v.x);
}

//计算矢量v与z轴正方向的夹角
inline RtLbsType Elevation(const Vector3D& v) {
	RtLbsType cosVal = sqrt((v.z) / v.Length());
	if (cosVal < -1) return PI;
	if (cosVal > 1) return 0.0;
	return acos(cosVal);
}

#endif
