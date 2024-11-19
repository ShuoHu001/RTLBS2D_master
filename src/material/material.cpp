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

Material::Material(const MaterialConfig& config)
{
	m_id = config.m_id;
	m_name = config.m_name;
	m_permittivity = config.m_permittivity;
	m_conductivity = config.m_conductivity;
	m_penetrationLoss = config.m_penetrationLoss;
	m_refractiveN = sqrt(m_permittivity * m_conductivity);
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
	Complex complexPermittivity;						/** @brief	����糣��	*/
	complexPermittivity.m_real = m_permittivity;
	complexPermittivity.m_imag = -m_conductivity / (TWO_PI * freq * VACUME_PERMITTIVITY);
	return complexPermittivity;
}

RtLbsType Material::GetRefractiveN() const
{
	return m_refractiveN;
}

