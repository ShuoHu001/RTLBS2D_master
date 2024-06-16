#include "angularelement.h"

AngularElement::AngularElement()
	: m_isValid(false)
	, m_category(0)
	, m_power(-FLT_MAX)
	, m_startTheta(0.0)
{
}

AngularElement::AngularElement(const AngularElement& element)
	: m_isValid(element.m_isValid)
	, m_category(element.m_category)
	, m_power(element.m_power)
	 , m_startTheta(element.m_startTheta)
{
}

AngularElement::~AngularElement()
{
}

AngularElement& AngularElement::operator=(const AngularElement& element)
{
	m_isValid = element.m_isValid;
	m_category = element.m_category;
	m_power = element.m_power;
	m_startTheta = element.m_startTheta;
	return *this;
}
