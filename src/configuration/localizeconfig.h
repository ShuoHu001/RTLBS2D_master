#ifndef RTLBS_LOCALIZECONFIG
#define RTLBS_LOCALIZECONFIG

#include "utility/enum.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include "managers/logmanager.h"
#include "configuration/localization/weightfactor.h"
#include "configuration/localization/solvingconfig.h"

const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE = "LBSMode";
const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD = "LBSMethod";
const std::string KEY_LOCALIZATIONCONFIG_HARDWAREMODE = "HardWareMode";
const std::string KEY_LOCALIZATIONCONFIG_THREADNUM = "ThreadNum";
const std::string KEY_LOCALIZATIONCONFIG_RAYLAUNCHHALFTHETA = "RayLaunchHalfTheta";
const std::string KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD = "GSPairClusterThreshold";
const std::string KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR = "WeightFactor";
const std::string KET_LOCALIZATIONCONFIG_SOLVINGCONFIG = "SolvingConfig";
const std::string KEY_LOCALIZATIONCONFIG_EXTENDAROUNDPOINTSTATE = "ExtendAroundPointState";
const std::string KEY_LOCALIZATIONCONFIG_SHIFTERRORMATRIXFILENAME = "ShiftErrorMatrixFile";
const std::string KEY_LOCALIZATIONCONFIG_HASSIMUERROR = "HasSimuError";

class LocalizeConfig {
public:
	LOCALIZATION_MODE m_lbsMode;				/** @brief	定位模式	*/
	LOCALIZATION_METHOD m_lbsMethod;			/** @brief	定位方法	*/
	HARDWAREMODE m_hardWareMode;				/** @brief	硬件计算模式	*/
	uint16_t m_threadNum;						/** @brief	多线程数量	*/
	RtLbsType m_rayLaunchHalfTheta;				/** @brief	射线发射半张角 单位°	*/
	RtLbsType m_gsPairClusterThreshold;			/** @brief	广义源对聚类门限，单位m	*/
	WeightFactor m_weightFactor;				/** @brief	权重因子	*/
	SolvingConfig m_solvingConfig;				/** @brief	方程求解配置	*/
	bool m_extendAroundPointState;				/** @brief	定位过程中扩展周边点状态	*/
	std::string m_shiftErrorMatrixFileName;		/** @brief	位移矩阵名称	*/
	bool m_hasSimuError;						/** @brief	是否含有仿真误差	*/
	 
public:
	LocalizeConfig();
	LocalizeConfig(const LocalizeConfig& config);
	~LocalizeConfig();
	LocalizeConfig& operator = (const LocalizeConfig& config);

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	bool IsValid() const;															//判定配置是否有效

};

#endif
