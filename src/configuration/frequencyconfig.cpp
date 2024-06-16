#include "frequencyconfig.h"

FrequencyConfig::FrequencyConfig()
	: m_centerFrequency(1.0e9)
	, m_bandWidth(0.0)
	, m_subCarrierNum(0)
{
}

FrequencyConfig::FrequencyConfig(const FrequencyConfig& config)
	: m_centerFrequency(config.m_centerFrequency)
	, m_bandWidth(config.m_bandWidth)
	, m_subCarrierNum(config.m_subCarrierNum)
{
}

FrequencyConfig::~FrequencyConfig()
{
}

FrequencyConfig& FrequencyConfig::operator=(const FrequencyConfig& config)
{
	m_centerFrequency = config.m_centerFrequency;
	m_bandWidth = config.m_bandWidth;
	m_subCarrierNum = config.m_subCarrierNum;
	return *this;
}

std::vector<RtLbsType> FrequencyConfig::GetFrequencyInformation() const
{
	std::vector<RtLbsType> freqs;
	unsigned subCarrierNum = m_subCarrierNum;
	if (subCarrierNum % 2 == 0) {			//保证频带数量为奇数
		subCarrierNum += 1;
	}

	//添加子载波频率
	if (m_bandWidth != 0) {
		unsigned sideNum = static_cast<unsigned>((subCarrierNum - 1) / 2);
		RtLbsType subBand = m_bandWidth / subCarrierNum;
		for (unsigned i = sideNum; i >= 1; --i) {
			RtLbsType freqSide1 = m_centerFrequency - i * subBand;
			freqs.push_back(freqSide1);
		}
		//添加中心频率
		freqs.push_back(m_centerFrequency);
		for (unsigned i = 1; i <= sideNum; ++i) {
			RtLbsType freqSide2 = m_centerFrequency + i * subBand;
			freqs.push_back(freqSide2);
		}
	}
	return freqs;
}

void FrequencyConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_CENTERFREQUENCY.c_str());												writer.Double(m_centerFrequency);
	writer.Key(KEY_BANDWIDTH.c_str());														writer.Double(m_bandWidth);
	writer.Key(KEY_SUBCARRIERNUM.c_str());													writer.Uint(m_subCarrierNum);
	writer.EndObject();
}

bool FrequencyConfig::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "FrequencyConfig: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_CENTERFREQUENCY.c_str())) {
		LOG_ERROR << "FrequencyConfig: missing " << KEY_CENTERFREQUENCY.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_BANDWIDTH.c_str())) {
		LOG_ERROR << "FrequencyConfig: missing " << KEY_BANDWIDTH.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_SUBCARRIERNUM.c_str())) {
		LOG_ERROR << "FrequencyConfig: missing " << KEY_SUBCARRIERNUM.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& centerFrequencyValue = value[KEY_CENTERFREQUENCY.c_str()];
	const rapidjson::Value& bandwidthValue = value[KEY_BANDWIDTH.c_str()];
	const rapidjson::Value& subCarrierNumValue = value[KEY_SUBCARRIERNUM.c_str()];

	if (!centerFrequencyValue.IsDouble()) {
		LOG_ERROR << "FrequencyConfig: " << KEY_CENTERFREQUENCY.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!bandwidthValue.IsDouble()) {
		LOG_ERROR << "FrequencyConfig: " << KEY_BANDWIDTH.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!subCarrierNumValue.IsUint()) {
		LOG_ERROR << "FrequencyConfig: " << KEY_SUBCARRIERNUM.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_centerFrequency = centerFrequencyValue.GetDouble();
	m_bandWidth = bandwidthValue.GetDouble();
	m_subCarrierNum = subCarrierNumValue.GetUint();

	return true;
}
