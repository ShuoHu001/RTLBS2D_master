#include "point3d.h"

Point3D::Point3D()
	: x(0.0)
	, y(0.0)
	, z(0.0)
	, m_pId(0)
{
}

Point3D::Point3D(RtLbsType _x, RtLbsType _y, RtLbsType _z)
	: x(_x)
	, y(_y)
	, z(_z)
	, m_pId(0)
{
}

Point3D::Point3D(RtLbsType _x, RtLbsType _y, RtLbsType _z, unsigned pid)
	: x(_x)
	, y(_y)
	, z(_z)
	, m_pId(pid)
{
}

Point3D::Point3D(const Point3D& p)
	: x(p.x)
	, y(p.y)
	, z(p.z)
	, m_pId(p.m_pId)
{
}

Point3D::Point3D(const Vector3D& v)
	: x(v.x)
	, y(v.y)
	, z(v.z)
	, m_pId(0)
{
}

Point3D::~Point3D()
{
}

Point3D Point3D::operator+(const Vector3D& v) const
{
	return Point3D(x + v.x, y + v.y, z + v.z);
}

Point3D Point3D::operator+(const Point3D& p) const
{
	return Point3D(x + p.x, y + p.y, z + p.z);
}

Point3D& Point3D::operator+=(const Vector3D& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

Point3D Point3D::operator-(const Vector3D& v) const
{
	return Point3D(x - v.x, y - v.y, z - v.z);
}

Vector3D Point3D::operator-(const Point3D& p) const
{
	return Vector3D(x - p.x, y - p.y, z - p.z);
}

Point3D& Point3D::operator-=(const Vector3D& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

Point3D Point3D::operator*(RtLbsType f) const
{
	return Point3D(x * f, y * f, z * f);
}

Point3D& Point3D::operator*=(RtLbsType f)
{
	x *= f;
	y *= f;
	z *= f;
	return *this;
}

Point3D Point3D::operator/(RtLbsType f) const
{
	if (f == 0)
		LOG_WARNING << "Can't be divided by zero!" << ENDL;
	return Point3D(x / f, y / f, z / f);
}

Point3D& Point3D::operator/=(RtLbsType f)
{
	if (f == 0)
		LOG_WARNING << "Can't be divided by zero!" << ENDL;
	x /= f;
	y /= f;
	z /= f;
	return *this;
}

Point3D& Point3D::operator=(const Point3D& p)
{
	x = p.x;
	y = p.y;
	z = p.z;
	return *this;
}

RtLbsType Point3D::operator[](unsigned id) const
{
	switch (id)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
	default:
		return 0.0;
	}
}

RtLbsType& Point3D::operator[](unsigned id)
{
	switch (id)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
	default:
		LOG_ERROR << "out of index" << CRASH;
		return x;
	}
}

bool Point3D::operator==(const Point3D& p) const
{
	return (x == p.x) && (y == p.y) && (z == p.z);
}

bool Point3D::operator!=(const Point3D& p) const
{
	return !(*this == p);
}

bool Point3D::operator<(const Point3D& p) const
{
	if (x != p.x)
		return x < p.x;
	if (y != p.y)
		return y < p.y;
	if (z != p.z)
		return z < p.z;
	return false; //两个值相同，返回false
}

bool Point3D::operator>(const Point3D& p) const
{
	if (x != p.x)
		return x > p.x;
	if (y != p.y)
		return y > p.y;
	if (z != p.z)
		return z > p.z;
	return false; //两个值相同，返回false
}

Point3D Point3D::operator*(const Euler& e) const
{
	//姿态角旋转，按照方位(phi)-俯仰(theta)-滚转(cosphi)的顺序进行
	RtLbsType cosPhi = cos(e.m_yaw);			/** @brief	方位角余弦	*/
	RtLbsType sinPhi = sin(e.m_yaw);			/** @brief	方位角正弦	*/
	RtLbsType cosTheta = cos(e.m_pitch);		/** @brief	俯仰角余弦	*/
	RtLbsType sinTheta = sin(e.m_pitch);		/** @brief	俯仰角正弦	*/
	RtLbsType cosPsi = cos(e.m_roll);			/** @brief	滚转角余弦	*/
	RtLbsType sinPsi = sin(e.m_roll);			/** @brief	滚转角正弦	*/
	double rotationMatrix[9] = {
		cosTheta * cosPsi, cosPsi * sinPhi * sinTheta - cosPhi * sinPsi,sinPhi * sinPsi + cosPhi * cosPsi * sinTheta,
		cosTheta * sinPsi, cosTheta * cosPsi + sinPhi * sinTheta * sinPsi, cosPsi * sinTheta * sinPsi - cosPsi * sinPhi,
		-1 * sinTheta,cosTheta * sinPhi,cosPhi * cosTheta };
	double vector[3] = { x, y, z };
	double retvl[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retvl, 1);

	return Point3D(retvl[0], retvl[1], retvl[2]);
}

Point3D& Point3D::operator*=(const Euler& e)
{
	//姿态角旋转，按照方位(phi)-俯仰(theta)-滚转(cosphi)的顺序进行
	RtLbsType cosPhi = cos(e.m_yaw);			/** @brief	方位角余弦	*/
	RtLbsType sinPhi = sin(e.m_yaw);			/** @brief	方位角正弦	*/
	RtLbsType cosTheta = cos(e.m_pitch);		/** @brief	俯仰角余弦	*/
	RtLbsType sinTheta = sin(e.m_pitch);		/** @brief	俯仰角正弦	*/
	RtLbsType cosPsi = cos(e.m_roll);			/** @brief	滚转角余弦	*/
	RtLbsType sinPsi = sin(e.m_roll);			/** @brief	滚转角正弦	*/
	double rotationMatrix[9] = {
		cosTheta * cosPsi, cosPsi * sinPhi * sinTheta - cosPhi * sinPsi,sinPhi * sinPsi + cosPhi * cosPsi * sinTheta,
		cosTheta * sinPsi, cosTheta * cosPsi + sinPhi * sinTheta * sinPsi, cosPsi * sinTheta * sinPsi - cosPsi * sinPhi,
		-1 * sinTheta,cosTheta * sinPhi,cosPhi * cosTheta };
	double vector[3] = { x, y, z };
	double retvl[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retvl, 1);

	x = retvl[0];
	y = retvl[1];
	z = retvl[2];

	return *this;
}

bool Point3D::IsZero() const
{
	if (x == 0.0 && y == 0.0 && z == 0.0)
		return true;
	return false;
}

Point3D& Point3D::Rotate(double phi)
{
	double sinPhi = sin(phi);
	double cosPhi = cos(phi);
	double matrix[9] = {
	cosPhi, -sinPhi, 0.0,
	sinPhi, cosPhi, 0.0,
	0.0, 0.0, 1.0
	};
	RtLbsType vector[3] = { x, y, z };
	RtLbsType retval[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, matrix, 3, vector, 1, 0.0, retval, 1);
	x = retval[0];
	y = retval[1];
	z = retval[2];
	return *this;
}

Point3D& Point3D::Rotate(double phi, double theta)
{
	// Define the rotation matrices
	double sinPhi = sin(phi);
	double cosPhi = cos(phi);
	double rotatePhi[9] = {
		cosPhi, -sinPhi, 0,
		sinPhi, cosPhi, 0,
		0, 0, 1
	};

	double sinTheta = sin(theta);
	double cosTheta = cos(theta);
	double rotateTheta[9] = {
		cosTheta, 0, sinTheta,
		0, 1, 0,
		-sinTheta, 0, cosTheta
	};

	double rotationMatrix[9];
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1.0, rotatePhi, 3, rotateTheta, 3, 0.0, rotationMatrix, 3);

	double vector[3] = { x, y, z };
	double result[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, result, 1);

	// Update the vector components
	x = result[0];
	y = result[1];
	z = result[2];

	return *this;
}

Point3D& Point3D::Rotate(const Vector3D& axis, double theta)
{
	double costheta = cos(theta);
	double sintheta = sin(theta);

	// Define the rotation matrix
	double rotationMatrix[9] = {
		costheta + axis.x * axis.x * (1 - costheta), axis.x * axis.y * (1 - costheta) - axis.z * sintheta, axis.x * axis.z * (1 - costheta) + axis.y * sintheta,
		axis.y * axis.x * (1 - costheta) + axis.z * sintheta, costheta + axis.y * axis.y * (1 - costheta), axis.y * axis.z * (1 - costheta) - axis.x * sintheta,
		axis.z * axis.x * (1 - costheta) - axis.y * sintheta, axis.z * axis.y * (1 - costheta) + axis.x * sintheta, costheta + axis.z * axis.z * (1 - costheta)
	};

	double vector[3] = { x, y, z };
	double retvl[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retvl, 1);

	// Update the vector components
	x = retvl[0];
	y = retvl[1];
	z = retvl[2];

	return *this;
}

void Point3D::Round(unsigned digit)
{
	std::stringstream ss;
	ss << std::fixed << std::setprecision(digit) << x;
	ss >> x;
	ss.clear();
	ss << std::fixed << std::setprecision(digit) << y;
	ss >> y;
	ss.clear();
	ss << std::fixed << std::setprecision(digit) << z;
	ss >> z;
	ss.clear();
	for (unsigned i = 0; i < 3; ++i) {
		RtLbsType& value = (*this)[i];
		std::stringstream ss;
		ss << std::fixed << std::setprecision(digit) << value;
		ss >> value;
		if (value == 0.0)
			value = 0.0;
	}
}

void Point3D::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key("x"); writer.Double(x);
	writer.Key("y"); writer.Double(y);
	writer.Key("z"); writer.Double(z);
	writer.EndObject();
}

bool Point3D::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		if (value.HasMember("x") &&
			value.HasMember("y") &&
			value.HasMember("z")) {
			const rapidjson::Value& xValue = value["x"];
			const rapidjson::Value& yValue = value["y"];
			const rapidjson::Value& zValue = value["z"];
			if (xValue.IsDouble() &&
				yValue.IsDouble() &&
				zValue.IsDouble()) {
				x = xValue.GetFloat();
				y = yValue.GetFloat();
				z = zValue.GetFloat();
				return true;
			}
		}
	}
	return false;
}

std::string Point3D::ToString() const
{
	std::stringstream ss;
	ss <<"(" << x << "," << y << "," << z << ")";
	return ss.str();
}

size_t Point3D::GetHash() const
{
	return util::Hash64(ToString());
}
