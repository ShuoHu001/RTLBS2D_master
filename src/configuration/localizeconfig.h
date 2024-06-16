#ifndef RTLBS_LOCALIZECONFIG
#define RTLBS_LOCALIZECONFIG

#include "utility/enum.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include "managers/logmanager.h"

const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE = "LBSMode";
const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD = "LBSMethod";

class LocalizeConfig:public Serializable {
public:
	LOCALIZATION_MODE m_lbsMode;				/** @brief	��λģʽ	*/
	LOCALIZATION_METHOD m_lbsMethod;			/** @brief	��λ����	*/
	 
public:
	LocalizeConfig();
	LocalizeConfig(const LocalizeConfig& config);
	~LocalizeConfig();
	LocalizeConfig& operator = (const LocalizeConfig& config);

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	bool IsValid() const;															//�ж������Ƿ���Ч

};

#endif
