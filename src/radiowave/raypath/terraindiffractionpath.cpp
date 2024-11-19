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
	RtLbsType routeLength = 0.0;					/** @brief	传播距离	*/
	for (int i = 0; i < m_nodes.size() - 1; ++i) {
		routeLength += (m_nodes[i + 1]->m_point - m_nodes[i]->m_point).Length();
	}
	return routeLength / LIGHT_VELOCITY_AIR;
}

RtLbsType TerrainDiffractionPath::GetPropagationLength() const
{
	RtLbsType routeLength = 0.0;					/** @brief	传播距离	*/
	for (int i = 0; i < m_nodes.size() - 1; ++i) {
		routeLength += (m_nodes[i + 1]->m_point - m_nodes[i]->m_point).Length();
	}
	return routeLength;
}

RtLbsType TerrainDiffractionPath::GetPhaseOffset(RtLbsType freq) const
{
	RtLbsType lamda = LIGHT_VELOCITY_AIR / freq;
	RtLbsType l = GetPropagationLength();				/** @brief	传播距离	*/
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
	RtLbsType radialVelocity = 0.0;					/** @brief	路径 径向上的相对速度 m/s	*/

	//计算发射机对径向速度的贡献
	Vector3D dirStart = (m_nodes[1]->m_point - m_nodes[0]->m_point).Normalize();		/** @brief	发射节点处的方向向量	*/
	RtLbsType startRadialVelocity = dirStart * txVelocity;								/** @brief	发射节点处的径向速度	*/
	//计算接收机对径向速度的共享
	Vector3D dirEnd = (m_nodes[m_nodes.size() - 1]->m_point - m_nodes[m_nodes.size() - 2]->m_point).Normalize();	/** @brief	指向接收节点处的方向向量	*/
	RtLbsType endRadialVelocity = dirEnd * rxVelocity;									/** @brief	接受节点处的径向速度	*/

	radialVelocity = startRadialVelocity + endRadialVelocity;							/** @brief	路径上的径向速度差值	*/

	//计算多普勒频移
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
	//循环每个节点进行更新node与前一个和后一个节点之间的距离
	for (int i = 1; i < m_nodes.size() - 1; ++i) {			//去除首尾点参数
		TerrainPathNode* preNode = m_nodes[i - 1];			/** @brief	前一节点	*/
		TerrainPathNode* curNode = m_nodes[i];				/** @brief	当前节点	*/
		curNode->m_s1 = (curNode->m_point - preNode->m_point).LengthXY();				//计算S1参数(距离前一个节点的有效距离)
		curNode->m_s2 = (curNode->m_point - m_nodes.back()->m_point).LengthXY();		//计算S2参数（距离接收节点的有效距离）
	}
	return;
}
