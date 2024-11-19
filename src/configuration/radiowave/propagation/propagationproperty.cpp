#include "propagationproperty.h"

HOST_DEVICE_FUNC PropagationProperty::PropagationProperty()
	: m_hasRelfection(true)
	, m_hasDiffraction(false)
	, m_hasTransmission(false)
	, m_hasEmpiricalTransmission(false)
	, m_hasScattering(false)
{
}

HOST_DEVICE_FUNC PropagationProperty::PropagationProperty(const PropagationProperty& property)
	: m_hasRelfection(property.m_hasRelfection)
	, m_hasDiffraction(property.m_hasDiffraction)
	, m_hasTransmission(property.m_hasTransmission)
	, m_hasEmpiricalTransmission(property.m_hasEmpiricalTransmission)
	, m_hasScattering(property.m_hasScattering)
{
}

HOST_DEVICE_FUNC PropagationProperty::PropagationProperty(bool hasReflection, bool hasDiffraction, bool hasTransmission, bool hasEmpiricalTransmission, bool hasScattering)
{
	m_hasRelfection = hasReflection;
	m_hasDiffraction = hasDiffraction;
	m_hasTransmission = hasTransmission;
	m_hasEmpiricalTransmission = hasEmpiricalTransmission;
	m_hasScattering = hasScattering;
}

HOST_DEVICE_FUNC PropagationProperty::~PropagationProperty()
{
}

HOST_DEVICE_FUNC PropagationProperty& PropagationProperty::operator=(const PropagationProperty& property)
{
	// TODO: 在此处插入 return 语句
	m_hasRelfection = property.m_hasRelfection;
	m_hasDiffraction = property.m_hasDiffraction;
	m_hasTransmission = property.m_hasTransmission;
	m_hasEmpiricalTransmission = property.m_hasEmpiricalTransmission;
	m_hasScattering = property.m_hasScattering;
	return *this;
}

HOST_DEVICE_FUNC bool PropagationProperty::operator==(const PropagationProperty& property) const
{
	if (m_hasRelfection != property.m_hasRelfection)
		return false;
	if (m_hasDiffraction != property.m_hasDiffraction)
		return false;
	if (m_hasTransmission != property.m_hasTransmission)
		return false;
	if (m_hasEmpiricalTransmission != property.m_hasEmpiricalTransmission)
		return false;
	if (m_hasScattering != property.m_hasScattering)
		return false;
	return true;

}

HOST_DEVICE_FUNC bool PropagationProperty::operator!=(const PropagationProperty& property) const
{
	return !(*this == property);
}

HOST_DEVICE_FUNC void PropagationProperty::Set(bool hasReflection, bool hasDiffraction, bool hasTransmission, bool hasEmpiricalTransmission, bool hasScattering)
{
	m_hasRelfection = hasReflection;
	m_hasDiffraction = hasDiffraction;
	m_hasTransmission = hasTransmission;
	m_hasEmpiricalTransmission = hasEmpiricalTransmission;
	m_hasScattering = hasScattering;
}

void PropagationProperty::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_PROPAGATIONPROPERTY_HASREFLECT.c_str()); writer.Bool(m_hasRelfection);
	writer.Key(KEY_PROPAGATIONPROPERTY_HASTRANSMIT.c_str()); writer.Bool(m_hasTransmission);
	writer.Key(KEY_PROPAGATIONPROPERTY_HASEMPIRICALTRANSMIT.c_str()); writer.Bool(m_hasEmpiricalTransmission);
	writer.Key(KEY_PROPAGATIONPROPERTY_HASDIFFRACT.c_str()); writer.Bool(m_hasDiffraction);
	writer.Key(KEY_PROPAGATIONPROPERTY_HASSCATTERING.c_str()); writer.Bool(m_hasScattering);
	writer.Key(KEY_PROPAGATIONPROPERTY_TERRAINDIFFRACTIONMODE.c_str()); SerializeEnum(m_terrainDiffractionMode, writer);
	writer.EndObject();
}

bool PropagationProperty::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		if (value.HasMember(KEY_PROPAGATIONPROPERTY_HASREFLECT.c_str()) &&
			value.HasMember(KEY_PROPAGATIONPROPERTY_HASTRANSMIT.c_str()) &&
			value.HasMember(KEY_PROPAGATIONPROPERTY_HASEMPIRICALTRANSMIT.c_str()) &&
			value.HasMember(KEY_PROPAGATIONPROPERTY_HASDIFFRACT.c_str()) &&
			value.HasMember(KEY_PROPAGATIONPROPERTY_HASSCATTERING.c_str()) &&
			value.HasMember(KEY_PROPAGATIONPROPERTY_TERRAINDIFFRACTIONMODE.c_str())) {
			const rapidjson::Value& hasReflectionValue = value[KEY_PROPAGATIONPROPERTY_HASREFLECT.c_str()];
			const rapidjson::Value& hasTransmissionValue = value[KEY_PROPAGATIONPROPERTY_HASTRANSMIT.c_str()];
			const rapidjson::Value& hasEmpiricalTransmissionValue = value[KEY_PROPAGATIONPROPERTY_HASEMPIRICALTRANSMIT.c_str()];
			const rapidjson::Value& hasDiffractionValue = value[KEY_PROPAGATIONPROPERTY_HASDIFFRACT.c_str()];
			const rapidjson::Value& hasScatteringValue = value[KEY_PROPAGATIONPROPERTY_HASSCATTERING.c_str()];
			const rapidjson::Value& terrainDifractionModeValue = value[KEY_PROPAGATIONPROPERTY_TERRAINDIFFRACTIONMODE.c_str()];
			if (hasReflectionValue.IsBool() &&
				hasTransmissionValue.IsBool() &&
				hasEmpiricalTransmissionValue.IsBool() &&
				hasDiffractionValue.IsBool() &&
				hasScatteringValue.IsBool() &&
				terrainDifractionModeValue.IsInt()) {
				m_hasRelfection = hasReflectionValue.GetBool();
				m_hasTransmission = hasTransmissionValue.GetBool();
				m_hasEmpiricalTransmission = hasEmpiricalTransmissionValue.GetBool();
				m_hasDiffraction = hasDiffractionValue.GetBool();
				m_hasScattering = hasScatteringValue.GetBool();
				return DeserializeEnum(m_terrainDiffractionMode, terrainDifractionModeValue);
			}
		}
	}
	return false;
}
