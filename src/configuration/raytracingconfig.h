#ifndef RTLBS_RAYTRACINGCONFIG
#define RTLBS_RAYTRACINGCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "managers/logmanager.h"
#include "utility/serializable.h"
#include "utility/enum.h"
#include "physical/limitinfo.h"

const std::string KEY_RAYTRACINGCONFIG_RAYNUM = "RayNum";
const std::string KEY_RAYTRACINGCONFIG_LIMITINFO = "LimitInfo";
const std::string KEY_RAYTRACINGCONFIG_RAYSPLITFLAG = "RaySplitFlag";
const std::string KEY_RAYTRACINGCONFIG_RAYSPLITRAIDUS = "RaySplitRadius";
const std::string KEY_RAYTRACINGCONFIG_SCENEACCELTYPE = "SceneAccelType";
const std::string KEY_RAYTRACINGCONFIG_HARDWAREMODE = "HardwareMode";
const std::string KEY_RAYTRACINGCONFIG_RAYLAUNCHMODE = "RayLaunchMode";
const std::string KEY_RAYTRACINGCONFIG_CPUTHREADNUM = "CPUThreadNum";

class RaytrcingConfig: public Serializable {
public:
	uint64_t m_rayNum;					/** @brief	��ʼ������������	*/
	LimitInfo m_limitInfo;				/** @brief	������Ϣ	*/
	bool m_raySplitFlag;				/** @brief	���߷��ѱ�־	*/
	RtLbsType m_raySplitRadius;			/** @brief	���߷��Ѱ뾶	*/
	ACCEL_TYPE m_sceneAccelType;		/** @brief	����ѡȡ�ļ��ٽṹ��	*/
	HARDWAREMODE m_hardwareMode;		/** @brief	Ӳ������ģʽ	*/
	RAYLAUNCHMODE m_rayLaunchMode;		/** @brief	���߷���ģʽ	*/
	uint16_t m_cpuThreadNum;			/** @brief	cpu������	*/
public:
	RaytrcingConfig();
	~RaytrcingConfig(){}
public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
