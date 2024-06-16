#include "vector3d.h"
#include "point3d.h"

Vector3D::Vector3D(bool normal)
	: x(0.0)
	, y(0.0)
	, z(0.0)
	, m_isNormal(false)
{
}

Vector3D::Vector3D(RtLbsType _x, RtLbsType _y, RtLbsType _z, bool normal)
	: x(_x)
	, y(_y)
	, z(_z)
	, m_isNormal(normal)
{
}

Vector3D::Vector3D(const Vector3D& v)
	: x(v.x)
	, y(v.y)
	, z(v.z)
	, m_isNormal(v.m_isNormal)
{
}

Vector3D::Vector3D(const Point3D& p)
	: x(p.x)
	, y(p.y)
	, z(p.z)
	, m_isNormal(false)
{
}

Vector3D Vector3D::operator+(const Vector3D& v) const
{
	return Vector3D(x + v.x, y + v.y, z + v.z);
}

Vector3D& Vector3D::operator+=(const Vector3D& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

Vector3D Vector3D::operator-(const Vector3D& v) const
{
	return Vector3D(x - v.x, y - v.y, z - v.z);
}

Vector3D& Vector3D::operator-=(const Vector3D& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

Vector3D Vector3D::operator*(RtLbsType f) const
{
	return Vector3D(x * f, y * f, z * f);
}

Vector3D& Vector3D::operator*=(RtLbsType f)
{
	x *= f;
	y *= f;
	z *= f;
	return *this;
}

Vector3D Vector3D::operator/(RtLbsType f) const
{
	if (f == 0)
		LOG_WARNING << "Can't be divided by zero." << ENDL;
	return Vector3D(x / f, y / f, z / f);
}

Vector3D& Vector3D::operator/=(RtLbsType f)
{
	if (f == 0)
		LOG_WARNING << "Can't be divided by zero." << ENDL;
	x /= f;
	y /= f;
	z /= f;
	return *this;
}

Vector3D& Vector3D::operator=(const Vector3D& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	m_isNormal = v.m_isNormal;
	return *this;
}

RtLbsType Vector3D::operator*(const Vector3D& v) const
{
	return x * v.x + y * v.y + z * v.z;
}

Vector3D Vector3D::operator*(const Euler& e) const
{
	//×ËÌ¬½ÇÐý×ª£¬°´ÕÕ·½Î»(phi)-¸©Ñö(theta)-¹ö×ª(cosphi)µÄË³Ðò½øÐÐ
	RtLbsType cosPhi = cos(e.m_yaw);			/** @brief	·½Î»½ÇÓàÏÒ	*/
	RtLbsType sinPhi = sin(e.m_yaw);			/** @brief	·½Î»½ÇÕýÏÒ	*/
	RtLbsType cosTheta = cos(e.m_pitch);		/** @brief	¸©Ñö½ÇÓàÏÒ	*/
	RtLbsType sinTheta = sin(e.m_pitch);		/** @brief	¸©Ñö½ÇÕýÏÒ	*/
	RtLbsType cosPsi = cos(e.m_roll);			/** @brief	¹ö×ª½ÇÓàÏÒ	*/
	RtLbsType sinPsi = sin(e.m_roll);			/** @brief	¹ö×ª½ÇÕýÏÒ	*/
	double rotationMatrix[9] = {
		cosTheta * cosPsi, cosPsi * sinPhi * sinTheta - cosPhi * sinPsi,sinPhi * sinPsi + cosPhi * cosPsi * sinTheta,
		cosTheta * sinPsi, cosTheta * cosPsi + sinPhi * sinTheta * sinPsi, cosPsi * sinTheta * sinPsi - cosPsi * sinPhi,
		-1 * sinTheta,cosTheta * sinPhi,cosPhi * cosTheta };
	double vector[3] = { x, y, z };
	double retvl[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, retvl, 1);

	return Vector3D(retvl[0], retvl[1], retvl[2]);
}

Vector3D& Vector3D::operator*=(const Euler& e)
{
	//×ËÌ¬½ÇÐý×ª£¬°´ÕÕ·½Î»(phi)-¸©Ñö(theta)-¹ö×ª(cosphi)µÄË³Ðò½øÐÐ
	RtLbsType cosPhi = cos(e.m_yaw);			/** @brief	·½Î»½ÇÓàÏÒ	*/
	RtLbsType sinPhi = sin(e.m_yaw);			/** @brief	·½Î»½ÇÕýÏÒ	*/
	RtLbsType cosTheta = cos(e.m_pitch);		/** @brief	¸©Ñö½ÇÓàÏÒ	*/
	RtLbsType sinTheta = sin(e.m_pitch);		/** @brief	¸©Ñö½ÇÕýÏÒ	*/
	RtLbsType cosPsi = cos(e.m_roll);			/** @brief	¹ö×ª½ÇÓàÏÒ	*/
	RtLbsType sinPsi = sin(e.m_roll);			/** @brief	¹ö×ª½ÇÕýÏÒ	*/
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

bool Vector3D::operator==(const Vector3D& v) const
{
	return (x == v.x) && (y == v.y) && (z == v.z);
}

bool Vector3D::operator!=(const Vector3D& v) const
{
	return !(*this == v);
}

RtLbsType Vector3D::operator[](unsigned id) const
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
		return 0;
	}
}

RtLbsType& Vector3D::operator[](unsigned id)
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

Vector3D Vector3D::operator-() const
{
	return Vector3D(-x, -y, -z, m_isNormal);
}

bool Vector3D::IsZero() const
{
	if (x == 0.0 && y == 0.0 && z == 0.0)
		return true;
	return false;
}

RtLbsType Vector3D::MaxComponent()
{
	return std::max({ x, y, z });
}

RtLbsType Vector3D::MinComponent()
{
	return std::min({ x, y, z });
}

int Vector3D::GetMaxId() const
{
	if (x >= y && x >= z)
		return 0;
	if (y >= x && y >= z)
		return 1;
	if (z >= x && z >= y)
		return 2;
	return 0;
}

int Vector3D::GetMinId() const
{
	if (x <= y && x <= z)
		return 0;
	if (y <= x && y <= z)
		return 1;
	if (z <= x && z <= y)
		return 2;
	return 0;
}

RtLbsType Vector3D::Length() const
{
	return static_cast<RtLbsType>(sqrt(x * x + y * y + z * z));
}

RtLbsType Vector3D::LengthXY() const
{
	return static_cast<RtLbsType>(sqrt(x * x + y * y));
}

RtLbsType Vector3D::SquaredLength() const
{
	return x * x + y * y + z * z;
}

RtLbsType Vector3D::SquaredLengthXY() const
{
	return x * x + y * y;
}

Vector3D& Vector3D::Normalize()
{
	RtLbsType len = Length();
	if (len == 0)
		LOG_WARNING << "Try to normalize a zero length vector!!" << ENDL;
	*this /= len;
	return *this;
}

RtLbsType Vector3D::Dot(const Vector3D& v) const
{
	return x * v.x + y * v.y + z * v.z;
}

RtLbsType Vector3D::AbsDot(const Vector3D& v) const
{
	RtLbsType r = Dot(v);
	return r < 0 ? -r : r;
}

Vector3D Vector3D::Cross(const Vector3D& v) const
{
	return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x, m_isNormal && v.m_isNormal);
}

Vector3D& Vector3D::Rotate(double phi)
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

Vector3D& Vector3D::Rotate(double phi, double theta)
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

Vector3D& Vector3D::Rotate(const Vector3D& axis, double theta)
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

Vector3D& Vector3D::RotatebyCos(double costheta)
{
	RtLbsType sintheta = sqrt(1 - costheta * costheta);
	double matrix[9] = {
	costheta, -sintheta, 0.0,
	sintheta, costheta, 0.0,
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

Vector3D& Vector3D::RotatebyCos(double costheta, double cosphi)
{
	double sintheta = sqrt(1 - costheta * costheta);
	double sinphi = sqrt(1 - sintheta * sintheta);
	// Define the rotation matrices
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

	double vector[3] = { x, y, z };
	double result[3];
	cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1.0, rotationMatrix, 3, vector, 1, 0.0, result, 1);

	// Update the vector components
	x = result[0];
	y = result[1];
	z = result[2];

	return *this;
}

Vector3D& Vector3D::RotatebyCos(const Vector3D& axis, double costheta)
{
	double sintheta = sqrt(1 - costheta * costheta);

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

RtLbsType Vector3D::Azimuth() const
{
	if (y < 0)
		return atan2(y, x) + TWO_PI;
	return atan2(y, x);
}

RtLbsType Vector3D::Elevation() const
{
	RtLbsType cosVal = z / Length();
	if (cosVal < -1)
		return PI;
	else if (cosVal > 1)
		return 0.0;
	else
		return acos(cosVal);
}

std::string Vector3D::ToString() const
{
	std::stringstream ss;
	ss << "(" << x << "," << y << "," << z << ")";
	return ss.str();
}

size_t Vector3D::GetHash() const
{
	return util::Hash64(ToString());
}

void Vector3D::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key("x"); writer.Double(x);
	writer.Key("y"); writer.Double(y);
	writer.Key("z"); writer.Double(z);
	writer.Key("IsNormal"); writer.Bool(m_isNormal);
	writer.EndObject();
}

bool Vector3D::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		if (value.HasMember("x") &&
			value.HasMember("y") &&
			value.HasMember("z") &&
			value.HasMember("IsNormal")) {
			const rapidjson::Value& xValue = value["x"];
			const rapidjson::Value& yValue = value["y"];
			const rapidjson::Value& zValue = value["z"];
			const rapidjson::Value& isNormalValue = value["IsNormal"];
			if (xValue.IsDouble() && yValue.IsDouble() &&
				zValue.IsDouble() && isNormalValue.IsBool()) {
				x = xValue.GetDouble();
				y = yValue.GetDouble();
				z = zValue.GetDouble();
				m_isNormal = isNormalValue.GetBool();
				return true;
			}
		}
	}
	return false;
}


