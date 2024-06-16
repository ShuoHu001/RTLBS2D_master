#include "point2d.h"


Point2D::Point2D()
    : x(0.0)
    , y(0.0)
    , distance(0.0)
{
}

Point2D::Point2D(RtLbsType _x, RtLbsType _y)
    : distance(0.0)
{
    x = _x;
    y = _y;
}

Point2D::Point2D(RtLbsType _x, RtLbsType _y, RtLbsType _distance)
    : x(_x)
    , y(_y)
    , distance(_distance)
{
}

Point2D::Point2D(const Point2D& p)
{
    x = p.x;
    y = p.y;
    distance = p.distance;
}

Point2D::Point2D(const Vector2D& v)
    : distance(0.0)
{
    x = v.x;
    y = v.y;
}

Point2D::~Point2D()
{
}

Point2D Point2D::operator+(const Vector2D& v) const
{
    return Point2D(x + v.x, y + v.y);
}

Point2D Point2D::operator+(const Point2D& p) const
{
    return Point2D(x + p.x, y + p.y);
}

Point2D& Point2D::operator+=(const Vector2D& v)
{
    x += v.x;
    y += v.y;
    return *this;
}

Point2D Point2D::operator-(const Vector2D& v) const
{
    return Point2D(x - v.x, y - v.y);
}

Vector2D Point2D::operator-(const Point2D& p) const
{
    return Point2D(x-p.x,y-p.y);
}

Point2D& Point2D::operator-=(const Vector2D& v)
{
    x -= v.x;
    y -= v.y;
    return *this;
}

Point2D Point2D::operator*(RtLbsType f) const
{
    return Point2D(x * f, y * f);
}

Point2D& Point2D::operator*=(RtLbsType f)
{
    x *= f;
    y *= f;
    return *this;
}

Point2D Point2D::operator/(RtLbsType f) const
{
    return Point2D(x/f,y/f);
}

Point2D& Point2D::operator/=(RtLbsType f)
{
    x /= f;
    y /= f;
    return *this;
}

Point2D& Point2D::operator=(const Point2D& p)
{
    x = p.x;
    y = p.y;
    return *this;
}

RtLbsType Point2D::operator[](unsigned id) const
{
	switch (id) {
	case 0: return x;
	case 1: return y;
	default: return 0.0f;
	}
}

RtLbsType& Point2D::operator[](unsigned id)
{
	switch (id) {
	case 0: return x;
	case 1: return y;
	default: return x;
	}
}

bool Point2D::operator==(const Point2D& p) const
{
    return abs(x - p.x)<EPSILON && abs(y - p.y)<EPSILON;
}

bool Point2D::operator!=(const Point2D& p) const
{
    return !(*this == p);
}

void Point2D::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
    writer.StartObject();
    writer.Key("x"); writer.Double(x);
    writer.Key("y"); writer.Double(y);
    writer.EndObject();
}

bool Point2D::Deserialize(const rapidjson::Value& value)
{
    if (value.IsObject()) {
        const rapidjson::Value& xValue = value["x"];
        const rapidjson::Value& yValue = value["y"];
        if (xValue.IsDouble() && yValue.IsDouble()) {
            x = xValue.GetFloat();
            y = yValue.GetFloat();
            return true;
        }
    }
    return false;
}


