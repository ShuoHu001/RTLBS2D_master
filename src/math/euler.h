#ifndef RTLBS_EULER
#define RTLBS_EULER

#include "rtlbs.h"
#include "managers/logmanager.h"
#include "utility/serializable.h"

class Vector3D;

const std::string KEY_EULER_ISRADIAN = "IsRadian";
const std::string KEY_EULER_YAW = "Yaw";
const std::string KEY_EULER_PITCH = "Pitch";
const std::string KEY_EULER_ROLL = "Roll";

//欧拉旋转角，旋转时按照偏航-俯仰-滚转的顺序进行
class Euler :public Serializable {
public:
	bool m_isRadian;				/** @brief	是否是弧度制,默认为弧度制	*/
	RtLbsType m_yaw;				/** @brief	偏航角 绕Z轴旋转	*/
	RtLbsType m_pitch;				/** @brief	俯仰角 绕X轴旋转	*/
	RtLbsType m_roll;				/** @brief	滚转角 绕Y轴旋转	*/

public:
	Euler();
	Euler(RtLbsType yaw, RtLbsType pitch, RtLbsType roll, bool isRadian = true);
	Euler(const Euler& euler);
	~Euler();
	Euler& operator = (const Euler& e);
	Euler operator + (const Euler& e) const;
	Euler& operator += (const Euler& e);
	Euler operator - (const Euler& e) const;
	Euler& operator -= (const Euler& e);
	Euler operator * (const RtLbsType& t) const;
	Euler& operator *= (const RtLbsType& t);
	Euler operator / (const RtLbsType& t) const;
	Euler& operator /= (const RtLbsType& t);
	bool operator == (const Euler& e) const;
	bool operator != (const Euler& e) const;
	RtLbsType operator [] (unsigned id) const;
	RtLbsType& operator [] (unsigned id);
	bool IsZero() const;													//是否是零角度
	Euler& ConvertToDegree();
	Euler& ConvertToRadian();
	Vector3D CalCartesianCoordinate(RtLbsType t) const;						//计算在笛卡尔坐标系下的坐标
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

inline Vector3D CalCartesianCoordinate(const Euler& posture, RtLbsType t);

#endif
