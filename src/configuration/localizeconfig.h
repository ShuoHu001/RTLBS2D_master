#ifndef RTLBS_LOCALIZECONFIG
#define RTLBS_LOCALIZECONFIG

#include "utility/enum.h"
#include "utility/serializable.h"
#include "utility/define.h"
#include "managers/logmanager.h"
#include "localization/weightfactor.h"

const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMODE = "LBSMode";
const std::string KEY_LOCALIZATIONCONFIG_LOCALIZATIONMETHOD = "LBSMethod";
const std::string KEY_LOCALIZATIONCONFIG_HARDWAREMODE = "HardWareMode";
const std::string KEY_LOCALIZATIONCONFIG_THREADNUM = "ThreadNum";
const std::string KEY_LOCALIZATIONCONFIG_RAYLAUNCHHALFTHETA = "RayLaunchHalfTheta";
const std::string KEY_LOCALIZATIONCONFIG_GSPAIRCLUSTERTHRESHOLD = "GSPairClusterThreshold";
const std::string KEY_LOCALIZATIONCONFIG_WEIGHTFACTOR = "WeightFactor";
const std::string KEY_LOCALIZATIONCONFIG_HASSIMUERROR = "HasSimuError";

class LocalizeConfig:public Serializable {
public:
	LOCALIZATION_MODE m_lbsMode;				/** @brief	��λģʽ	*/
	LOCALIZATION_METHOD m_lbsMethod;			/** @brief	��λ����	*/
	HARDWAREMODE m_hardWareMode;				/** @brief	Ӳ������ģʽ	*/
	uint16_t m_threadNum;						/** @brief	���߳�����	*/
	RtLbsType m_rayLaunchHalfTheta;				/** @brief	���߷�����Ž� ��λ��	*/
	RtLbsType m_gsPairClusterThreshold;			/** @brief	����Դ�Ծ������ޣ���λm	*/
	WeightFactor m_weightFactor;				/** @brief	Ȩ������	*/
	bool m_hasSimuError;						/** @brief	�Ƿ��з������	*/
	 
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
