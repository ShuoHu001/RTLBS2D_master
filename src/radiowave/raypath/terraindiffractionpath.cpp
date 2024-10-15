#include "terraindiffractionpath.h"

TerrainPathNode::TerrainPathNode()
	: m_clearanceHeight(0.0)
	, m_s1(0.0)
	, m_s2(0.0)
	, m_ridge(nullptr)
{
}

TerrainPathNode::TerrainPathNode(const Point3D& edgePoint)
	: m_point(edgePoint)
	, m_clearanceHeight(0.0)
	, m_s1(0.0)
	, m_s2(0.0)
	, m_ridge(nullptr)
{
}

TerrainPathNode::TerrainPathNode(const TerrainRidge* ridge)
	: m_s1(0.0)
	, m_s2(0.0)
	, m_ridge(ridge)
{
	m_point = *ridge->m_peak->m_point3d;
	m_clearanceHeight = ridge->m_relativeHeight;
}

TerrainPathNode::TerrainPathNode(Point3D& p, RtLbsType& clearanceHeight, RtLbsType& sPs, RtLbsType& sPe)
	: m_point(p)
	, m_clearanceHeight(clearanceHeight)
	, m_s1(sPs)
	, m_s2(sPe)
	, m_ridge(nullptr)
{
}

TerrainPathNode::TerrainPathNode(const TerrainRidge* prevRidge, const TerrainRidge* curRidge, const TerrainRidge* nextRidge)
	: m_ridge(curRidge)
{
	m_point = *curRidge->m_peak->m_point3d;
	m_clearanceHeight = curRidge->m_relativeHeight;
	if (prevRidge == nullptr) {
		m_s1 = 0.0;
		m_s2 = abs(nextRidge->m_peak->m_point2d[0] - curRidge->m_peak->m_point2d[0]);
		return;
	}
	if (nextRidge == nullptr) {
		m_s1 = abs(curRidge->m_peak->m_point2d[0] - prevRidge->m_peak->m_point2d[0]);
		m_s2 = 0.0;
		return;
	}
	m_s1 = abs(curRidge->m_peak->m_point2d[0] - prevRidge->m_peak->m_point2d[0]);
	m_s2 = abs(nextRidge->m_peak->m_point2d[0] - curRidge->m_peak->m_point2d[0]);
}

TerrainPathNode::TerrainPathNode(const TerrainPathNode& node)
	: m_point(node.m_point)
	, m_clearanceHeight(node.m_clearanceHeight)
	, m_s1(node.m_s1)
	, m_s2(node.m_s2)
	, m_ridge(node.m_ridge)
{
}

TerrainPathNode::~TerrainPathNode()
{
}

TerrainPathNode& TerrainPathNode::operator=(const TerrainPathNode& node)
{
	m_point = node.m_point;
	m_clearanceHeight = node.m_clearanceHeight;
	m_s1 = node.m_s1;
	m_s2 = node.m_s2;
	return *this;
}

bool TerrainPathNode::operator==(const TerrainPathNode& node) const
{
	if (m_point != node.m_point)
		return false;
	if (m_clearanceHeight != node.m_clearanceHeight)
		return false;
	if (m_s1 != node.m_s1)
		return false;
	if (m_s2 != node.m_s2)
		return false;
	return true;
}

bool TerrainPathNode::operator!=(const TerrainPathNode& node) const
{
	return !(*this == node);
}


TerrainDiffractionPath::TerrainDiffractionPath()
	: m_terrainDiffractionMode(DIFFRACTIONMODE_PICQUENARD)
{

}

TerrainDiffractionPath::~TerrainDiffractionPath()
{
	for (int i = 0; i < m_nodes.size(); ++i) {
		delete m_nodes[i];
	}
	m_nodes.clear();
}

RtLbsType TerrainDiffractionPath::GetPropagationTime() const
{
	RtLbsType routeLength = 0.0;					/** @brief	��������	*/
	for (int i = 0; i < m_nodes.size() - 1; ++i) {
		routeLength += (m_nodes[i + 1]->m_point - m_nodes[i]->m_point).Length();
	}
	return routeLength / LIGHT_VELOCITY_AIR;
}

RtLbsType TerrainDiffractionPath::GetPropagationLength() const
{
	RtLbsType routeLength = 0.0;					/** @brief	��������	*/
	for (int i = 0; i < m_nodes.size() - 1; ++i) {
		routeLength += (m_nodes[i + 1]->m_point - m_nodes[i]->m_point).Length();
	}
	return routeLength;
}

RtLbsType TerrainDiffractionPath::GetPhaseOffset(RtLbsType freq) const
{
	RtLbsType lamda = LIGHT_VELOCITY_AIR / freq;
	RtLbsType l = GetPropagationLength();				/** @brief	��������	*/
	RtLbsType phaseShift = fmod(l, lamda) / lamda * TWO_PI;
	return phaseShift;
}

RtLbsType TerrainDiffractionPath::GetAngleofDeparture_Phi() const
{
	Vector3D routeStart = m_nodes[1]->m_point - m_nodes[0]->m_point;
	return routeStart.Azimuth();
}

RtLbsType TerrainDiffractionPath::GetAngleofDeparture_Theta() const
{
	Vector3D routeStart = m_nodes[1]->m_point - m_nodes[0]->m_point;
	return routeStart.Elevation();
}

RtLbsType TerrainDiffractionPath::GetAngleofArrival_Phi() const
{
	Vector3D routeEnd = m_nodes[m_nodes.size() - 1]->m_point - m_nodes[m_nodes.size() - 2]->m_point;
	return routeEnd.Azimuth();
}

RtLbsType TerrainDiffractionPath::GetAngleofArrival_Theta() const
{
	Vector3D routeEnd = m_nodes[m_nodes.size() - 1]->m_point - m_nodes[m_nodes.size() - 2]->m_point;
	return routeEnd.Elevation();
}

RtLbsType TerrainDiffractionPath::CalDopplerShift(RtLbsType freq, const Vector3D& txVelocity, const Vector3D& rxVelocity)
{
	if (m_nodes.size() < 2)
		return 0.0;
	RtLbsType radialVelocity = 0.0;					/** @brief	·�� �����ϵ�����ٶ� m/s	*/

	//���㷢����Ծ����ٶȵĹ���
	Vector3D dirStart = (m_nodes[1]->m_point - m_nodes[0]->m_point).Normalize();		/** @brief	����ڵ㴦�ķ�������	*/
	RtLbsType startRadialVelocity = dirStart * txVelocity;								/** @brief	����ڵ㴦�ľ����ٶ�	*/
	//������ջ��Ծ����ٶȵĹ���
	Vector3D dirEnd = (m_nodes[m_nodes.size() - 1]->m_point - m_nodes[m_nodes.size() - 2]->m_point).Normalize();	/** @brief	ָ����սڵ㴦�ķ�������	*/
	RtLbsType endRadialVelocity = dirEnd * rxVelocity;									/** @brief	���ܽڵ㴦�ľ����ٶ�	*/

	radialVelocity = startRadialVelocity + endRadialVelocity;							/** @brief	·���ϵľ����ٶȲ�ֵ	*/

	//���������Ƶ��
	RtLbsType dopplerShift = radialVelocity / LIGHT_VELOCITY_AIR * freq;
	return dopplerShift;
}

void TerrainDiffractionPath::OuputRaypath(std::ofstream& stream) const
{
	stream << m_nodes.size() << " ";
	for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		const Point3D& p = (*it)->m_point;
		stream << p.x << " " << p.y << " " << p.z << " ";
	}
	stream << std::endl;
}

void TerrainDiffractionPath::RectifySParameters()
{
	//ѭ��ÿ���ڵ���и���node��ǰһ���ͺ�һ���ڵ�֮��ľ���
	for (int i = 1; i < m_nodes.size() - 1; ++i) {			//ȥ����β�����
		TerrainPathNode* preNode = m_nodes[i - 1];			/** @brief	ǰһ�ڵ�	*/
		TerrainPathNode* curNode = m_nodes[i];				/** @brief	��ǰ�ڵ�	*/
		curNode->m_s1 = (curNode->m_point - preNode->m_point).LengthXY();				//����S1����(����ǰһ���ڵ����Ч����)
		curNode->m_s2 = (curNode->m_point - m_nodes.back()->m_point).LengthXY();		//����S2������������սڵ����Ч���룩
	}
	return;
}
