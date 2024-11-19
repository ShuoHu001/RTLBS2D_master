#include "vector2d.h"
#include "point2d.h"

HOST_DEVICE_FUNC Vector2D::Vector2D(bool normal)
	: x(-FLT_MAX)
	, y(-FLT_MAX)
	, m_bNormal(normal)
{
}

HOST_DEVICE_FUNC Vector2D::Vector2D(double radian, bool normal)
{
	x = static_cast<RtLbsType>(cos(radian));
	y = static_cast<RtLbsType>(sin(radian));
	m_bNormal = normal;
}


HOST_DEVICE_FUNC Vector2D::Vector2D(RtLbsType _x, RtLbsType _y, bool normal)
{
	x = _x;
	y = _y;
	m_bNormal = normal;
}

HOST_DEVICE_FUNC Vector2D::Vector2D(const Vector2D& v)
{
	x = v.x;
	y = v.y;
	m_bNormal = v.m_bNormal;
}

HOST_DEVICE_FUNC Vector2D::Vector2D(const Point2D& p)
{
	x = p.x;
	y = p.y;
	m_bNormal = false;
}

HOST_DEVICE_FUNC Vector2D Vector2D::operator+(const Vector2D& v) const
{
	return Vector2D(x + v.x, y + v.y);
}

HOST_DEVICE_FUNC Vector2D& Vector2D::operator+=(const Vector2D& v)
{
	// TODO: 在此处插入 return 语句
	x += v.x;
	y += v.y;
	return *this;
}

HOST_DEVICE_FUNC Vector2D Vector2D::operator-(const Vector2D& v) const
{
	return Vector2D(x - v.x, y - v.y);
}

HOST_DEVICE_FUNC Vector2D& Vector2D::operator-=(const Vector2D& v)
{
	x -= v.x;
	y -= v.y;
	return *this;
}

HOST_DEVICE_FUNC Vector2D Vector2D::operator*(RtLbsType f) const
{
	return Vector2D(f * x, f * y);
}

HOST_DEVICE_FUNC Vector2D& Vector2D::operator*=(RtLbsType f)
{
	x *= f;
	y *= f;
	return *this;
}

HOST_DEVICE_FUNC Vector2D Vector2D::operator/(RtLbsType f) const
{
	if (f == 0) {
#ifdef __CUDA_ARCH__
		printf("Vector2D 除0错误\n");
#else
		LOG_ERROR << "除0错误" << ENDL;
		CRASH;
#endif
	}
	return Vector2D(x / f, y / f);
}

HOST_DEVICE_FUNC Vector2D& Vector2D::operator/=(RtLbsType f)
{
	if (f == 0) {
#ifdef __CUDA_ARCH__
		printf("Vector2D 除0错误\n");
#else
		LOG_ERROR << "除0错误" << ENDL;
		CRASH;
#endif
	}
	x /= f;
	y /= f;
	return *this;
}

HOST_DEVICE_FUNC Vector2D& Vector2D::operator=(const Vector2D& v)
{
	x = v.x;
	y = v.y;
	m_bNormal = v.m_bNormal;
	return *this;
}

HOST_DEVICE_FUNC RtLbsType Vector2D::operator*(const Vector2D& v) const
{
	return x * v.x + y * v.y;
}

HOST_DEVICE_FUNC bool Vector2D::operator==(const Vector2D& v) const
{
	return (x == v.x) && (y == v.y);
}

HOST_DEVICE_FUNC bool Vector2D::operator!=(const Vector2D& v) const
{
	return !(*this == v);
}

HOST_DEVICE_FUNC RtLbsType Vector2D::operator[](unsigned id) const
{
	switch (id) {
		case 0: return x;
		case 1: return y;
		default: return 0.0f;
	}
}

HOST_DEVICE_FUNC RtLbsType& Vector2D::operator[](unsigned id)
{
	switch (id) {
		case 0: return x;
		case 1: return y;
		default: return x;
	}
}

HOST_DEVICE_FUNC Vector2D Vector2D::operator-() const
{
	return Vector2D(-x, -y, m_bNormal);
}

HOST_DEVICE_FUNC RtLbsType Vector2D::MaxComponent()
{
	return x > y ? x : y;
}

HOST_DEVICE_FUNC RtLbsType Vector2D::MinComponent()
{
	return x < y ? x : y;
}

HOST_DEVICE_FUNC RtLbsType Vector2D::Length() const
{
	RtLbsType len = static_cast<RtLbsType>(sqrt(x * x + y * y));
	return len;
}

HOST_DEVICE_FUNC RtLbsType Vector2D::SquaredLength() const
{
	RtLbsType lenSquared = x * x + y * y;
	return lenSquared;
}

HOST_DEVICE_FUNC Vector2D& Vector2D::Normalize()
{
	RtLbsType len = static_cast<RtLbsType>(sqrt(x * x + y * y));
	if (len > 0.0) {
		RtLbsType invLen = 1.0 / len;
		x *= invLen;
		y *= invLen;
	}
	m_bNormal = true;//归一化之后为单位向量
	return *this;
}


HOST_DEVICE_FUNC Vector2D& Vector2D::Rotate(double theta)
{
	// 计算旋转后的坐标
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	RtLbsType newX = static_cast<RtLbsType>(x * cosTheta - y * sinTheta);
	RtLbsType newY = static_cast<RtLbsType>(x * sinTheta + y * cosTheta);
	x = newX;
	y = newY;
	return *this;
}


HOST_DEVICE_FUNC Vector2D& Vector2D::RotatebyCos(double& costheta)
{
	double sintheta = sqrt(1 - costheta * costheta);
	RtLbsType newX = static_cast<RtLbsType>(x * costheta - y * sintheta);
	RtLbsType newY = static_cast<RtLbsType>(x * sintheta + y * costheta);
	x = newX;
	y = newY;
	return *this;
}

HOST_DEVICE_FUNC RtLbsType Vector2D::GetTheta() const
{
	RtLbsType theta = std::atan2(y, x);
	if (theta < 0) {						//若角度小于0，代表落入第三、四象限，需要转换为正角度阈值
		theta += TWO_PI;
	}
	return theta;
}

void Vector2D::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.String("x"); writer.Double(x);
	writer.String("y"); writer.Double(y);
	writer.String("m_bNormal"); writer.Bool(m_bNormal);
	writer.EndObject();
}

bool Vector2D::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		const rapidjson::Value& xValue = value["x"];
		const rapidjson::Value& yValue = value["y"];
		const rapidjson::Value& m_bNormalValue = value["m_bNormal"];
		if (xValue.IsDouble() && yValue.IsDouble() && m_bNormalValue.IsBool()) {
			x = xValue.GetFloat();
			y = yValue.GetFloat();
			m_bNormal = m_bNormalValue.GetBool();
			return true;
		}
	}
	return false;
}


