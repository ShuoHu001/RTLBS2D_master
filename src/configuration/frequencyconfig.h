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
	RtLbsType m_centerFrequency;				/** @brief	����Ƶ��	*/
	RtLbsType m_bandWidth;						/** @brief	Ƶ�����	*/
	unsigned m_subCarrierNum;					/** @brief	���ز�����,ż��	*/

public:
	FrequencyConfig();
	FrequencyConfig(const FrequencyConfig& config);
	~FrequencyConfig();
	FrequencyConfig& operator = (const FrequencyConfig& config);
	std::vector<RtLbsType> GetFrequencyInformation() const;												//��ȡƵ����Ϣ
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

private:
	const std::string KEY_CENTERFREQUENCY = "CenterFrequency";
	const std::string KEY_BANDWIDTH = "BandWidth";
	const std::string KEY_SUBCARRIERNUM = "SubCarrierNum";
};

#endif
