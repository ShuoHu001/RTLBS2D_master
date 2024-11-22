#include "tdoa_residual.h"

TDOAResidual::TDOAResidual()
	: m_x1(0.0)
	, m_y1(0.0)
	, m_xi(0.0)
	, m_yi(0.0)
	, m_timeDiff(0.0)
	, m_weight(1.0)
{
}

TDOAResidual::TDOAResidual(const GeneralSource* refSource, const GeneralSource* dataSource, RtLbsType weight)
{
	m_x1 = refSource->m_position.x;
	m_y1 = refSource->m_position.y;
	m_xi = dataSource->m_position.x;
	m_yi = dataSource->m_position.y;
	m_timeDiff = dataSource->m_sensorData.m_timeDiff;
	m_weight = weight;
}

TDOAResidual::TDOAResidual(const TDOAResidual& residual)
	: m_x1(residual.m_x1)
	, m_y1(residual.m_y1)
	, m_xi(residual.m_xi)
	, m_yi(residual.m_yi)
	, m_timeDiff(residual.m_timeDiff)
	, m_weight(residual.m_weight)
{
}

TDOAResidual::~TDOAResidual()
{
}

TDOAResidual& TDOAResidual::operator=(const TDOAResidual& residual)
{
	m_x1 = residual.m_x1;
	m_y1 = residual.m_y1;
	m_xi = residual.m_xi;
	m_yi = residual.m_yi;
	m_timeDiff = residual.m_timeDiff;
	m_weight = residual.m_weight;
	return *this;
}

void TDOAResidual::Init(const GeneralSource* refSource, const GeneralSource* dataSource, RtLbsType weight)
{
	m_x1 = refSource->m_position.x;
	m_y1 = refSource->m_position.y;
	m_xi = dataSource->m_position.x;
	m_yi = dataSource->m_position.y;
	m_timeDiff = dataSource->m_sensorData.m_timeDiff;
	m_weight = weight;
}

double TDOAResidual::GetResidual(const double* position) const
{
	double x = position[0];							/** @brief	预测点 x坐标	*/
	double y = position[1];							/** @brief	预测点 y坐标	*/
	double d1 = std::sqrt((x - m_x1) * (x - m_x1) + (y - m_y1) * (y - m_y1));
	double di = std::sqrt((x - m_xi) * (x - m_xi) + (y - m_yi) * (y - m_yi));
	return ((di - d1) / LIGHT_VELOCITY_AIR - m_timeDiff ) * m_weight* 1e9;				//ns 时间差
}

RtLbsType TDOAResidual::GetWeight() const
{
	return m_weight;
}

void TDOAResidual::SetWeight(RtLbsType weight)
{
	m_weight = weight;
}