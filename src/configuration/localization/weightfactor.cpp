#include "weightfactor.h"

WeightFactor::WeightFactor()
	: m_phiWeight(1.0)
	, m_timeWeight(1.0)
	, m_powerWeight(1.0)
{
}

WeightFactor::WeightFactor(RtLbsType phiWeight, RtLbsType timeWeight, RtLbsType powerWeight)
	: m_phiWeight(phiWeight)
	, m_timeWeight(timeWeight)
	, m_powerWeight(powerWeight)
{
}

WeightFactor::WeightFactor(const WeightFactor& weight)
	: m_phiWeight(weight.m_phiWeight)
	, m_timeWeight(weight.m_timeWeight)
	, m_powerWeight(weight.m_powerWeight)
{
}

WeightFactor::~WeightFactor()
{
}

WeightFactor& WeightFactor::operator=(const WeightFactor& factor)
{
	m_phiWeight = factor.m_phiWeight;
	m_timeWeight = factor.m_timeWeight;
	m_powerWeight = factor.m_powerWeight;
	return *this;
}

void WeightFactor::InitAOAWeight()
{
	RtLbsType tw = m_phiWeight + m_powerWeight;
	m_phiWeight /= tw;
	m_powerWeight /= tw;
}

void WeightFactor::InitAOATDOAWeight()
{
	RtLbsType tw = m_timeWeight + m_phiWeight + m_powerWeight;
	m_timeWeight /= tw;
	m_phiWeight /= tw;
	m_powerWeight /= tw;
}

void WeightFactor::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_WEIGHTFACTOR_PHIWEIGHT.c_str());											writer.Double(m_phiWeight);
	writer.Key(KEY_WEIGHTFACTOR_TIMEWEIGHT.c_str());										writer.Double(m_timeWeight);
	writer.Key(KET_WEIGHTFACTOR_POWERWEIGHT.c_str());										writer.Double(m_powerWeight);
	writer.EndObject();
}

bool WeightFactor::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "WeightFactor: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_WEIGHTFACTOR_PHIWEIGHT.c_str())) {
		LOG_ERROR << "WeightFactor: missing " << KEY_WEIGHTFACTOR_PHIWEIGHT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_WEIGHTFACTOR_TIMEWEIGHT.c_str())) {
		LOG_ERROR << "WeightFactor: missing " << KEY_WEIGHTFACTOR_TIMEWEIGHT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KET_WEIGHTFACTOR_POWERWEIGHT.c_str())) {
		LOG_ERROR << "WeightFactor: missing " << KET_WEIGHTFACTOR_POWERWEIGHT.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& phiWeightValue = value[KEY_WEIGHTFACTOR_PHIWEIGHT.c_str()];
	const rapidjson::Value& timeWeightValue = value[KEY_WEIGHTFACTOR_TIMEWEIGHT.c_str()];
	const rapidjson::Value& powerWeightValue = value[KET_WEIGHTFACTOR_POWERWEIGHT.c_str()];

	if (!phiWeightValue.IsDouble()) {
		LOG_ERROR << "WeightFactor: " << KEY_WEIGHTFACTOR_PHIWEIGHT.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!timeWeightValue.IsDouble()) {
		LOG_ERROR << "WeightFactor: " << KEY_WEIGHTFACTOR_TIMEWEIGHT.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!powerWeightValue.IsDouble()) {
		LOG_ERROR << "WeightFactor: " << KET_WEIGHTFACTOR_POWERWEIGHT.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_phiWeight = phiWeightValue.GetDouble();
	m_timeWeight = timeWeightValue.GetDouble();
	m_powerWeight = powerWeightValue.GetDouble();

	return true;
}
