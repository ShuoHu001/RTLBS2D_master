#include "material.h"

Material::Material()
	: m_id(-1)
	, m_name("")
	, m_permittivity(0.0)
	, m_conductivity(0.0)
	, m_penetrationLoss(5.0)
{
}

Material::Material(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity)
	: m_id(matId)
	, m_name(name)
	, m_permittivity(permittivity)
	, m_conductivity(conductivity)
	, m_penetrationLoss(5.0)
{
	m_refractiveN = sqrt(m_permittivity * m_conductivity);
}

Material::Material(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity, RtLbsType penetractionLoss)
	: m_id(matId)
	, m_name(name)
	, m_permittivity(permittivity)
	, m_conductivity(conductivity)
	, m_penetrationLoss(penetractionLoss)
{
	m_refractiveN = sqrt(m_permittivity * m_conductivity);
}

Material::Material(const Material& mat)
	: m_id(mat.m_id)
	, m_name(mat.m_name)
	, m_permittivity(mat.m_permittivity)
	, m_conductivity(mat.m_conductivity)
	, m_penetrationLoss(mat.m_penetrationLoss)
	, m_refractiveN(mat.m_refractiveN)
{
}

Material::~Material()
{
}

Material& Material::operator=(const Material& mat)
{
	m_id = mat.m_id;
	m_name = mat.m_name;
	m_permittivity = mat.m_permittivity;
	m_conductivity = mat.m_conductivity;
	m_penetrationLoss = mat.m_penetrationLoss;
	m_refractiveN = mat.m_refractiveN;
	return *this;
}

bool Material::operator==(const Material& mat) const
{
	if (m_permittivity == mat.m_permittivity &&
		m_conductivity == mat.m_conductivity)
		return true;
	return false;
}

bool Material::operator!=(const Material& mat) const
{
	return !(*this == mat);
}

Complex Material::GetComplexForm(RtLbsType freq) const
{
	Complex complexPermittivity;						/** @brief	¸´½éµç³£Êý	*/
	complexPermittivity.m_real = m_permittivity;
	complexPermittivity.m_imag = -m_conductivity / (TWO_PI * freq * VACUME_PERMITTIVITY);
	return complexPermittivity;
}

RtLbsType Material::GetRefractiveN() const
{
	return m_refractiveN;
}

void Material::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_MATERIAL_ID.c_str()); writer.Int(m_id);
	writer.Key(KEY_MATERIAL_NAME.c_str()); writer.String(m_name.c_str());
	writer.Key(KEY_MATERIAL_PERMITTIVITY.c_str()); writer.Double(m_permittivity);
	writer.Key(KEY_MATERIAL_CONDUCTIVITY.c_str()); writer.Double(m_conductivity);
	writer.Key(KEY_MATERIAL_PENETRATIONLOSS.c_str()); writer.Double(m_penetrationLoss);
	writer.EndObject();
}

bool Material::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		if (value.HasMember(KEY_MATERIAL_ID.c_str()) &&
			value.HasMember(KEY_MATERIAL_NAME.c_str()) &&
			value.HasMember(KEY_MATERIAL_PERMITTIVITY.c_str()) &&
			value.HasMember(KEY_MATERIAL_CONDUCTIVITY.c_str()) &&
			value.HasMember(KEY_MATERIAL_PENETRATIONLOSS.c_str())) {
			const rapidjson::Value& idValue = value[KEY_MATERIAL_ID.c_str()];
			const rapidjson::Value& nameValue = value[KEY_MATERIAL_NAME.c_str()];
			const rapidjson::Value& permittivityValue = value[KEY_MATERIAL_PERMITTIVITY.c_str()];
			const rapidjson::Value& conductivityValue = value[KEY_MATERIAL_CONDUCTIVITY.c_str()];
			const rapidjson::Value& penetrationLossValue = value[KEY_MATERIAL_PENETRATIONLOSS.c_str()];
			if (idValue.IsInt() &&
				nameValue.IsString() &&
				permittivityValue.IsDouble() &&
				conductivityValue.IsDouble() &&
				penetrationLossValue.IsDouble()) {
				m_id = idValue.GetInt();
				m_name = nameValue.GetString();
				m_permittivity = permittivityValue.GetDouble();
				m_conductivity = conductivityValue.GetDouble();
				m_penetrationLoss = penetrationLossValue.GetDouble();
				m_refractiveN = sqrt(m_permittivity * m_conductivity);
				return true;
			}
		}
	}
	return false;
}

