#include "antennapattern.h"

AntennaPattern::AntennaPattern()
	: m_patternId(-1)
	, m_patternType(PATTERN2D)
	, m_freq(0.0)
	, m_eInterval2D(0.0)
	, m_hInterval2D(0.0)
	, m_eInterval3D(0.0)
	, m_hInterval3D(0.0)
{
}

AntennaPattern::~AntennaPattern()
{
}

RtLbsType AntennaPattern::GetGain(RtLbsType azimuth, RtLbsType elevation) const
{
	//这里将采用两种方法进行拟合天线方向图，分别对应于两篇论文
	return 0.0;
}

void AntennaPattern::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key("patternId"); writer.Int(m_patternId);
	writer.Key("patternType"); SerializeEnum(m_patternType, writer);
	writer.Key("freq"); writer.Double(m_freq);
	writer.Key("eInterval2D"); writer.Double(m_eInterval2D);
	writer.Key("hInterval2D"); writer.Double(m_hInterval2D);
	writer.Key("eData"); SerializeArray(m_eData, writer);
	writer.Key("hData"); SerializeArray(m_hData, writer);
	writer.Key("eInterval3D"); writer.Double(m_eInterval3D);
	writer.Key("hInterval3D"); writer.Double(m_hInterval3D);
	writer.Key("ehData"); SerializeArray(m_ehData, writer);
	writer.EndObject();
}

bool AntennaPattern::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		const rapidjson::Value& patternIdValue = value["patternId"];
		const rapidjson::Value& patternTypeValue = value["patternType"];
		const rapidjson::Value& freqValue = value["freq"];
		const rapidjson::Value& eInterval2DValue = value["eInterval2D"];
		const rapidjson::Value& hInterval2DValue = value["hInterval2D"];
		const rapidjson::Value& eDataValue = value["eData"];
		const rapidjson::Value& hDataValue = value["hData"];
		const rapidjson::Value& eInterval3DValue = value["eInterval3D"];
		const rapidjson::Value& hInterval3DValue = value["hInterval3D"];
		const rapidjson::Value& ehDataValue = value["ehData"];
		if (patternIdValue.IsInt() && patternTypeValue.IsInt() &&
			freqValue.IsDouble() && eInterval2DValue.IsDouble() &&
			hInterval2DValue.IsDouble() && eDataValue.IsArray() &&
			hDataValue.IsArray() && eInterval3DValue.IsDouble() &&
			hInterval3DValue.IsDouble() && ehDataValue.IsArray()) {
			m_patternId = patternIdValue.GetInt();
			m_freq = freqValue.GetDouble();
			m_eInterval2D = eInterval2DValue.GetDouble();
			m_hInterval2D = hInterval2DValue.GetDouble();
			m_eInterval3D = eInterval3DValue.GetDouble();
			m_hInterval3D = hInterval3DValue.GetDouble();
			if (DeserializeEnum(m_patternType, patternTypeValue) &&
				DeserializeArray(m_eData, eDataValue) &&
				DeserializeArray(m_hData, hDataValue) &&
				DeserializeArray(m_ehData, ehDataValue)) {
				return true;
			}
		}
	}
	return false;
}

