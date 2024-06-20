#include "limitinfo.h"

LimitInfo::LimitInfo()
	: m_limitTotal(3)
	, m_limitReflect(1)
	, m_limitTransmit(0)
	, m_limitDiffract(0)
	, m_limitScatter(0)
	, m_depth(0)
{
}

LimitInfo::LimitInfo(uint8_t limitTotal, uint8_t limitReflect, uint8_t limitTransmit, uint8_t limitDiffract, uint8_t limitScatter)
	: m_limitTotal(limitTotal)
	, m_limitReflect(limitReflect)
	, m_limitTransmit(limitTransmit)
	, m_limitDiffract(limitDiffract)
	, m_limitScatter(limitScatter)
	, m_depth(0)
{
}

LimitInfo::LimitInfo(const LimitInfo& info)
	: m_limitTotal(info.m_limitTotal)
	, m_limitReflect(info.m_limitReflect)
	, m_limitTransmit(info.m_limitTransmit)
	, m_limitDiffract(info.m_limitDiffract)
	, m_limitScatter(info.m_limitScatter)
	, m_depth(info.m_depth)
{
}

LimitInfo::~LimitInfo()
{
}

LimitInfo& LimitInfo::operator=(const LimitInfo& info)
{
	m_limitTotal = info.m_limitTotal;
	m_limitReflect = info.m_limitReflect;
	m_limitTransmit = info.m_limitTransmit;
	m_limitDiffract = info.m_limitDiffract;
	m_limitScatter = info.m_limitScatter;
	m_depth = info.m_depth;
	return *this;
}

bool LimitInfo::operator==(const LimitInfo& info) const
{
	if (m_limitTotal != info.m_limitTotal)
		return false;
	if (m_limitReflect != info.m_limitReflect)
		return false;
	if (m_limitTransmit != info.m_limitTransmit)
		return false;
	if (m_limitDiffract != info.m_limitDiffract)
		return false;
	if (m_limitScatter != info.m_limitScatter)
		return false;
	if (m_depth != info.m_depth)
		return false;
	return true;
}

bool LimitInfo::operator!=(const LimitInfo& info) const
{
	return !(*this == info);
}

bool LimitInfo::IsValid() const
{
	if (m_limitTotal == 0)								//若总限制数为0,则表明该限制信息无效
		return false;
	return true;
}

void LimitInfo::MinusReflectLimit()
{
	if (m_limitTotal == 0 || m_limitReflect == 0)
		return;
	m_limitReflect--;
	m_limitTotal--;
	m_depth++;
}

void LimitInfo::MinusTransmitLimit()
{
	if (m_limitTotal == 0 || m_limitTransmit == 0)
		return;
	m_limitTransmit--;
	m_limitTotal--;
	m_depth++;
}

void LimitInfo::MinusDiffractLimit()
{
	if (m_limitTotal == 0 || m_limitDiffract == 0)
		return;
	m_limitDiffract--;
	m_limitTotal--;
	m_depth++;
}

void LimitInfo::MinusScatterLimit()
{
	if (m_limitTotal == 0 || m_limitScatter == 0)
		return;
	m_limitScatter--;
	m_limitTotal--;
	m_depth++;
}

HOST_DEVICE_FUNC void LimitInfo::SetETranLimitInfo()
{
	//经验透射后不再进行任何的常规传播机制
	m_limitReflect = 0;
	m_limitTransmit = 0;
	m_limitDiffract = 0;
	m_limitScatter = 0;
}

EndStopInfo LimitInfo::CalEndStopInfo() const
{
	EndStopInfo info;
	if (m_limitTotal == 0) {
		info = { true, true, true, true };
	}
	else {
		info.EndStopReflect = (m_limitReflect != 0) ? false : true;
		info.EndStopTransmit = (m_limitTransmit != 0) ? false : true;
		info.EndStopDiffract = (m_limitDiffract != 0) ? false : true;
		info.EndStopScatter = (m_limitScatter != 0) ? false : true;
	}
	return info;
}

void LimitInfo::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const
{
	writer.StartObject();
	writer.Key(KEY_LIMITINFO_LIMITTOTAL.c_str());											writer.Uint(m_limitTotal);
	writer.Key(KEY_LIMITINFO_LIMITREFLECT.c_str());											writer.Uint(m_limitReflect);
	writer.Key(KEY_LIMITINFO_LIMITTRANSMIT.c_str());										writer.Uint(m_limitTransmit);
	writer.Key(KEY_LIMITINFO_LIMITDIFFRACT.c_str());										writer.Uint(m_limitDiffract);
	writer.Key(KEY_LIMITINFO_LIMITSCATTER.c_str());											writer.Uint(m_limitScatter);
	writer.EndObject();
}

bool LimitInfo::Deserialize(const rapidjson::Value& value)
{
	if (!value.IsObject()) {
		LOG_ERROR << "LimitInfo: not a json object." << ENDL;
		return false;
	}

	if (!value.HasMember(KEY_LIMITINFO_LIMITTOTAL.c_str())) {
		LOG_ERROR << "LimitInfo: missing  " << KEY_LIMITINFO_LIMITTOTAL.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LIMITINFO_LIMITREFLECT.c_str())) {
		LOG_ERROR << "LimitInfo: missing  " << KEY_LIMITINFO_LIMITREFLECT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LIMITINFO_LIMITTRANSMIT.c_str())) {
		LOG_ERROR << "LimitInfo: missing  " << KEY_LIMITINFO_LIMITTRANSMIT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LIMITINFO_LIMITDIFFRACT.c_str())) {
		LOG_ERROR << "LimitInfo: missing  " << KEY_LIMITINFO_LIMITDIFFRACT.c_str() << ENDL;
		return false;
	}
	if (!value.HasMember(KEY_LIMITINFO_LIMITSCATTER.c_str())) {
		LOG_ERROR << "LimitInfo: missing  " << KEY_LIMITINFO_LIMITSCATTER.c_str() << ENDL;
		return false;
	}

	const rapidjson::Value& limitTotalValue = value[KEY_LIMITINFO_LIMITTOTAL.c_str()];
	const rapidjson::Value& limitReflectValue = value[KEY_LIMITINFO_LIMITREFLECT.c_str()];
	const rapidjson::Value& limitTransmitValue = value[KEY_LIMITINFO_LIMITTRANSMIT.c_str()];
	const rapidjson::Value& limitDiffractValue = value[KEY_LIMITINFO_LIMITDIFFRACT.c_str()];
	const rapidjson::Value& limitScatterValue = value[KEY_LIMITINFO_LIMITSCATTER.c_str()];

	if (!limitTotalValue.IsUint()) {
		LOG_ERROR << "SimConfig: " << KEY_LIMITINFO_LIMITTOTAL.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!limitReflectValue.IsUint()) {
		LOG_ERROR << "SimConfig: " << KEY_LIMITINFO_LIMITREFLECT.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!limitTransmitValue.IsUint()) {
		LOG_ERROR << "SimConfig: " << KEY_LIMITINFO_LIMITTRANSMIT.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!limitDiffractValue.IsUint()) {
		LOG_ERROR << "SimConfig: " << KEY_LIMITINFO_LIMITDIFFRACT.c_str() << ", wrong value format." << ENDL;
		return false;
	}
	if (!limitScatterValue.IsUint()) {
		LOG_ERROR << "SimConfig: " << KEY_LIMITINFO_LIMITSCATTER.c_str() << ", wrong value format." << ENDL;
		return false;
	}

	m_limitTotal = limitTotalValue.GetUint();
	m_limitReflect = limitReflectValue.GetUint();
	m_limitTransmit = limitTransmitValue.GetUint();
	m_limitDiffract = limitDiffractValue.GetUint();
	m_limitScatter = limitScatterValue.GetUint();

	return true;
}
