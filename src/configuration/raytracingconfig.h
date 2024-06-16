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
	uint64_t m_rayNum;					/** @brief	初始的总射线数量	*/
	LimitInfo m_limitInfo;				/** @brief	限制信息	*/
	bool m_raySplitFlag;				/** @brief	射线分裂标志	*/
	RtLbsType m_raySplitRadius;			/** @brief	射线分裂半径	*/
	ACCEL_TYPE m_sceneAccelType;		/** @brief	场景选取的加速结构体	*/
	HARDWAREMODE m_hardwareMode;		/** @brief	硬件工作模式	*/
	RAYLAUNCHMODE m_rayLaunchMode;		/** @brief	射线发射模式	*/
	uint16_t m_cpuThreadNum;			/** @brief	cpu并行数	*/
public:
	RaytrcingConfig();
	~RaytrcingConfig(){}
public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
