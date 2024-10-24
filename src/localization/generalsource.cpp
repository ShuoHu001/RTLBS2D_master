#include "generalsource.h"

GeneralSource::GeneralSource()
	: m_isValid(true)
	, m_type(NODE_INIT)
	, m_depth(0)
	, m_wCount(0)
	, m_weight(0.0)
	, m_segment(nullptr)
	, m_wedge(nullptr)
	, m_phiRepeatCount(1)
{
}

GeneralSource::GeneralSource(const GeneralSource& s)
	: m_isValid(s.m_isValid)
	, m_type(s.m_type)
	, m_depth(s.m_depth)
	, m_sensorData(s.m_sensorData)
	, m_wCount(s.m_wCount)
	, m_weight(s.m_weight)
	, m_position(s.m_position)
	, m_nodePosition(s.m_nodePosition)
	, m_segment(s.m_segment)
	, m_wedge(s.m_wedge)
	, m_phiRepeatCount(s.m_phiRepeatCount)
	, m_originPathNode(s.m_originPathNode)
{
}

GeneralSource::~GeneralSource()
{
}

GeneralSource& GeneralSource::operator=(const GeneralSource& s)
{
	m_isValid = s.m_isValid;
	m_type = s.m_type;
	m_depth = s.m_depth;
	m_sensorData = s.m_sensorData;
	m_wCount = s.m_wCount;
	m_weight = s.m_weight;
	m_position = s.m_position;
	m_nodePosition = s.m_nodePosition;
	m_segment = s.m_segment;
	m_wedge = s.m_wedge;
	m_phiRepeatCount = s.m_phiRepeatCount;
	m_originPathNode = s.m_originPathNode;
	return *this;
}

bool GeneralSource::IsValid() const
{
	if (m_wCount == 0 || !m_isValid) {
		return false;
	}
	return true;
}

void GeneralSource::NormalizedWeight(RtLbsType maxWeight)
{
	m_weight = m_weight / maxWeight;
}

void GeneralSource::UpdateEvenPhiValue()
{
	if (m_phiRepeatCount > 1) {
		m_sensorData.m_phi /= m_phiRepeatCount;
	}
}

void GeneralSource::Output2File(std::ofstream& stream) const
{
	stream << m_weight << "\t";
	stream << m_position.x << "\t" << m_position.y << "\t";
	stream << m_sensorData.m_time << "\t";
	stream << m_sensorData.m_timeDiff << "\t";
	stream << m_sensorData.m_phi << "\t";
	stream << m_sensorData.m_power << "\n";
}

std::string GeneralSource::ToString() const
{
	int segmentId = 0;
	if (m_segment != nullptr) {
		segmentId = m_segment->m_id;
	}
	std::stringstream ss;
	if (m_type == NODE_REFL) {
		ss << m_type << "," << m_depth << "," << segmentId << "," << m_sensorData.m_sensorId << "," << m_sensorData.m_id;
	}
	else {
		ss << m_type << "," << m_depth << "," << segmentId << "," << m_sensorData.m_sensorId << "," << m_sensorData.m_id << "," << m_position.x << "," << m_position.y;
	}
	
	return ss.str();
}

size_t GeneralSource::GetHash() const
{
	return util::Hash64(ToString());
}

