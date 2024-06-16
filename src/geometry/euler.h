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

//ŷ����ת�ǣ���תʱ����ƫ��-����-��ת��˳�����
class Euler :public Serializable {
public:
	bool m_isRadian;				/** @brief	�Ƿ��ǻ�����,Ĭ��Ϊ������	*/
	RtLbsType m_yaw;				/** @brief	ƫ���� ��Z����ת	*/
	RtLbsType m_pitch;				/** @brief	������ ��X����ת	*/
	RtLbsType m_roll;				/** @brief	��ת�� ��Y����ת	*/

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
	bool IsZero() const;													//�Ƿ�����Ƕ�
	Euler& ConvertToDegree();
	Euler& ConvertToRadian();
	Vector3D CalCartesianCoordinate(RtLbsType t) const;						//�����ڵѿ�������ϵ�µ�����
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

inline Vector3D CalCartesianCoordinate(const Euler& posture, RtLbsType t);

#endif
