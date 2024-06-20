#ifndef RTLBS_LOCALIZECONFIG
#define RTLBS_LOCALIZECONFIG

#include "utility/enum.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include "managers/logmanager.h"

const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE = "LBSMode";
const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD = "LBSMethod";
const std::string KEY_LOCALIZATIONCONFIG_HARDWAREMODE = "HardWareMode";
const std::string KEY_LOCALIZATIONCONFIG_THREADNUM = "ThreadNum";
const std::string KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD = "GSPairClusterThreshold";

class LocalizeConfig:public Serializable {
public:
	LOCALIZATION_MODE m_lbsMode;				/** @brief	定位模式	*/
	LOCALIZATION_METHOD m_lbsMethod;			/** @brief	定位方法	*/
	HARDWAREMODE m_hardWareMode;				/** @brief	硬件计算模式	*/
	uint16_t m_threadNum;						/** @brief	多线程数量	*/
	RtLbsType m_gsPairClusterThreshold;			/** @brief	广义源对聚类门限，单位m	*/
	 
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
