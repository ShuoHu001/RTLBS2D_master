#ifndef RTLBS_FREQUENCYCONFIG
#define RTLBS_FREQUENCYCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "rapidjson/prettywriter.h"
#include "managers/logmanager.h"

class FrequencyConfig :public Serializable {
public:
	RtLbsType m_centerFrequency;				/** @brief	中心频率	*/
	RtLbsType m_bandWidth;						/** @brief	频带间隔	*/
	unsigned m_subCarrierNum;					/** @brief	子载波数量,偶数	*/

public:
	FrequencyConfig();
	FrequencyConfig(const FrequencyConfig& config);
	~FrequencyConfig();
	FrequencyConfig& operator = (const FrequencyConfig& config);
	std::vector<RtLbsType> GetFrequencyInformation() const;												//获取频带信息
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

private:
	const std::string KEY_CENTERFREQUENCY = "CenterFrequency";
	const std::string KEY_BANDWIDTH = "BandWidth";
	const std::string KEY_SUBCARRIERNUM = "SubCarrierNum";
};

#endif
