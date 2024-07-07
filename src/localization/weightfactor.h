#ifndef RTLBS_WEIGHTFACTOR
#define RTLBS_WEIGHTFACTOR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"

const std::string KEY_WEIGHTFACTOR_PHIWEIGHT = "PhiWeight";
const std::string KEY_WEIGHTFACTOR_TIMEWEIGHT = "TimeWeight";
const std::string KET_WEIGHTFACTOR_POWERWEIGHT = "PowerWeight";

class WeightFactor {
public:
	RtLbsType m_phiWeight;								/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType m_timeWeight;								/** @brief	ʱ��Ȩ��	*/
	RtLbsType m_powerWeight;							/** @brief	����Ȩ��	*/

public:
	WeightFactor();
	WeightFactor(RtLbsType phiWeight, RtLbsType timeWeight, RtLbsType powerWeight);
	WeightFactor(const WeightFactor& weight);
	~WeightFactor();
	WeightFactor& operator = (const WeightFactor& factor);
	void InitAOAWeight();
	void InitAOATDOAWeight();
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
