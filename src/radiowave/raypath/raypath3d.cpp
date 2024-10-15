#include "raypath3d.h"
#include "gpu/raypathgpu.h"

RayPath3D::RayPath3D()
	: m_propagationLength(0.0)
	, m_energyRatio(0.0)
	, m_containRefract(false)
	, m_containSplitNode(false)
	, m_isValid(true)
	, m_type(RAYPATH_COMMON)
	, m_angularSpectrumCategoryId(-1)
{
}

RayPath3D::RayPath3D(const RayPath& path, const Point3D& tx, const Point3D& rx)
	: m_propagationLength(0.0)
	, m_energyRatio(0.0)
	, m_containRefract(false)
	, m_containSplitNode(false)
	, m_isValid(true)
	, m_type(RAYPATH_COMMON)
	, m_angularSpectrumCategoryId(-1)
{
	m_nodes.resize(path.m_nodes.size());
	//判别两种转换状态
	if (abs(tx.z - rx.z) <= EPSILON) {															//tx和rx位置相同，采用同一种高度进行修正
		RtLbsType h = tx.z;																		/** @brief	高度	*/
		for (int i = 0; i < m_nodes.size(); ++i) {
			m_nodes[i] = new PathNode3D();
			m_nodes[i]->ConvertBy(*path.m_nodes[i], h);
			if (m_nodes[i]->m_type == NODE_TRANIN ||
				m_nodes[i]->m_type == NODE_TRANOUT ||
				m_nodes[i]->m_type == NODE_ETRANIN ||
				m_nodes[i]->m_type == NODE_ETRANOUT) {												//若包含透射节点，则该路径为透射路径
				m_containRefract = true;
			}
		}
	}
	else {																						//tx和rx位置不同，采用斜率修正模式
		RtLbsType deltaH = rx.z - tx.z;															/** @brief	高度差	*/
		RtLbsType ltotal = path.m_nodes.back()->m_ft;												/** @brief	传播总长	*/
		RtLbsType hMin = std::min(tx.z, rx.z);													/** @brief	最小高度值	*/
		for (int i = 0; i < m_nodes.size(); ++i) {
			RtLbsType ratio = path.m_nodes[i]->m_ft / ltotal;										/** @brief	等比比例	*/
			RtLbsType h = hMin + ratio * deltaH;
			m_nodes[i] = new PathNode3D();
			m_nodes[i]->ConvertBy(*path.m_nodes[i], h);
			if (m_nodes[i]->m_type == NODE_TRANIN ||
				m_nodes[i]->m_type == NODE_TRANOUT ||
				m_nodes[i]->m_type == NODE_ETRANIN ||
				m_nodes[i]->m_type == NODE_ETRANOUT) {												//若包含透射节点，则此条路径为透射路径
				m_containRefract = true;
			}
		}
	}
}

RayPath3D::RayPath3D(const RayPath& path, RtLbsType height)
	: m_propagationLength(0.0)
	, m_energyRatio(0.0)
	, m_containRefract(false)
	, m_containSplitNode(false)
	, m_isValid(true)
	, m_type(RAYPATH_COMMON)
	, m_angularSpectrumCategoryId(-1)
{
	m_nodes.resize(path.m_nodes.size());
	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i] = new PathNode3D();
		m_nodes[i]->ConvertBy(*path.m_nodes[i], height);
		if (m_nodes[i]->m_type == NODE_TRANIN ||
			m_nodes[i]->m_type == NODE_TRANOUT ||
			m_nodes[i]->m_type == NODE_ETRANIN ||
			m_nodes[i]->m_type == NODE_ETRANOUT) {												//若包含透射节点，则该路径为透射路径
			m_containRefract = true;
		}
	}
}

RayPath3D::RayPath3D(const RayPathGPU& path, const Point3D& tx, const Point3D& rx, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges)
	: m_propagationLength(0.0)
	, m_energyRatio(0.0)
	, m_containRefract(false)
	, m_containSplitNode(false)
	, m_isValid(true)
	, m_type(RAYPATH_COMMON)
	, m_angularSpectrumCategoryId(-1)
{
	int nodesCount = static_cast<int>(path.m_nodes.size());
	m_nodes.resize(nodesCount);														//节点分配内存

	//注意，由于GPU多径计算与CPU结果的不同，主要体现在数据结果是倒序，因此需要特殊处理

	if (abs(tx.z - rx.z) < EPSILON) {															//若tx和rx位置相同，则采用同一高度进行修复
		RtLbsType h = tx.z;																		/** @brief	高度	*/
		for (int i = 0; i < nodesCount; ++i) {
			m_nodes[i] = new PathNode3D();
			m_nodes[i]->ConvertBy(*path.m_nodes[nodesCount - 1 - i], h, segments, wedges);
			if (m_nodes[i]->m_type == NODE_TRANIN ||
				m_nodes[i]->m_type == NODE_TRANOUT ||
				m_nodes[i]->m_type == NODE_ETRANIN ||
				m_nodes[i]->m_type == NODE_ETRANOUT) {												//若包含透射节点，则该路径为透射路径
				m_containRefract = true;
			}
		}
	}
	else {																						//tx和rx位置不同，采用斜率修正模式
		RtLbsType deltaH = rx.z - tx.z;															/** @brief	高度差	*/
		RtLbsType ltotal = path.m_nodes.front()->m_ft;									/** @brief	传播总长	*/
		RtLbsType hMin = std::min(tx.z, rx.z);													/** @brief	最小高度值	*/
		for (int i = 0; i < nodesCount; ++i) {
			RtLbsType ratio = path.m_nodes[nodesCount - 1 - i]->m_ft / ltotal;							/** @brief	等比比例	*/
			RtLbsType h = hMin + ratio * deltaH;
			m_nodes[i] = new PathNode3D();
			m_nodes[i]->ConvertBy(*path.m_nodes[nodesCount - 1 - i], h, segments, wedges);
			if (m_nodes[i]->m_type == NODE_TRANIN ||
				m_nodes[i]->m_type == NODE_TRANOUT ||
				m_nodes[i]->m_type == NODE_ETRANIN ||
				m_nodes[i]->m_type == NODE_ETRANOUT) {												//若包含透射节点，则此条路径为透射路径
				m_containRefract = true;
			}
		}
	}

}

RayPath3D::RayPath3D(const RayPath3D& path)
	: m_propagationLength(path.m_propagationLength)
	, m_energyRatio(path.m_energyRatio)
	, m_containRefract(path.m_containRefract)
	, m_containSplitNode(path.m_containSplitNode)
	, m_isValid(path.m_isValid)
	, m_type(path.m_type)
	, m_angularSpectrumCategoryId(path.m_angularSpectrumCategoryId)
{
}

RayPath3D::~RayPath3D()
{
	for (auto it = m_nodes.begin(); it != m_nodes.end();++it) {
		delete* it;
	}
}

RayPath3D& RayPath3D::operator=(const RayPath3D& path)
{
	m_nodes.resize(path.m_nodes.size());
	for (int i = 0; i < path.m_nodes.size(); ++i) {
		m_nodes[i] = path.m_nodes[i];
	}
	m_propagationLength = path.m_propagationLength;
	m_energyRatio = path.m_energyRatio;
	m_containRefract = path.m_containRefract;
	m_containSplitNode = path.m_containSplitNode;
	m_isValid = path.m_isValid;
	m_type = path.m_type;
	m_angularSpectrumCategoryId = path.m_angularSpectrumCategoryId;
	return *this;
}

void RayPath3D::Init(const RayPath& path, RtLbsType height)
{
	m_nodes.resize(path.m_nodes.size());
	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i] = new PathNode3D();
		m_nodes[i]->ConvertBy(*path.m_nodes[i], height);
		if (m_nodes[i]->m_type == NODE_TRANIN ||
			m_nodes[i]->m_type == NODE_TRANOUT ||
			m_nodes[i]->m_type == NODE_ETRANIN ||
			m_nodes[i]->m_type == NODE_ETRANOUT) {												//若包含透射节点，则该路径为透射路径
			m_containRefract = true;
		}
	}
}

void RayPath3D::ReverseRayPath()
{
	if (m_nodes.size() < 2) {
		return;
	}
	const PATHNODETYPE beginNodeType = m_nodes.front()->m_type;
	const PATHNODETYPE endNodeType = m_nodes.back()->m_type;
	std::reverse(m_nodes.begin(), m_nodes.end());
	m_nodes.front()->m_type = beginNodeType;
	m_nodes.back()->m_type = endNodeType;
	
}

void RayPath3D::ConvertByRayPath(const RayPath& path, const Point3D& tx, const Point3D& rx)
{
	m_containRefract = path.m_bContainRefract;
	m_type = RAYPATH_COMMON;
	m_angularSpectrumCategoryId = path.m_angularSpectrumCategoryId;
	m_nodes.resize(path.m_nodes.size());
	m_nodes.front()->m_point = tx;
	m_nodes.back()->m_point = rx;
	//判别两种转换状态
	if (abs(tx.z - rx.z) <= EPSILON) {															//tx和rx位置相同，采用同一种高度进行修正
		RtLbsType h = tx.z;																		/** @brief	高度	*/
		for (int i = 1; i < m_nodes.size() - 1; ++i) {
			m_nodes[i]->ConvertBy(*path.m_nodes[i], h);
		}
	}
	else {																						//tx和rx位置不同，采用斜率修正模式
		RtLbsType deltaH = rx.z - tx.z;															/** @brief	高度差	*/
		RtLbsType ltotal = path.m_nodes.back()->m_ft;												/** @brief	传播总长	*/
		RtLbsType hMin = std::min(tx.z, rx.z);													/** @brief	最小高度值	*/
		for (int i = 1; i < m_nodes.size() - 1; ++i) {
			RtLbsType ratio = path.m_nodes[i]->m_ft / ltotal;										/** @brief	等比比例	*/
			RtLbsType h = hMin + ratio * deltaH;
			m_nodes[i]->ConvertBy(*path.m_nodes[i], h);
		}
	}
}

void RayPath3D::Union(PathNode3D* node)
{
	m_nodes.push_back(node);
}

RtLbsType RayPath3D::GetPropagationTime() const
{
	RtLbsType routeLength = 0.0;					/** @brief	传播距离	*/
	for (int i = 0; i < m_nodes.size() - 1; ++i) {
		routeLength += (m_nodes[i + 1]->m_point - m_nodes[i]->m_point).Length();
	}
	return routeLength / LIGHT_VELOCITY_AIR;
}

RtLbsType RayPath3D::GetPropagationLength() const
{
	RtLbsType routeLength = 0.0;					/** @brief	传播距离	*/
	for (int i = 0; i < m_nodes.size() - 1; ++i) {
		routeLength += (m_nodes[i + 1]->m_point - m_nodes[i]->m_point).Length();
	}
	return routeLength;
}

RtLbsType RayPath3D::GetPhaseOffset(RtLbsType freq) const
{
	RtLbsType lamda = LIGHT_VELOCITY_AIR / freq;
	RtLbsType l = GetPropagationLength();						/** @brief	传播距离	*/
	RtLbsType phaseShift = fmod(l, lamda) / lamda * TWO_PI;		/** @brief	相位漂移	*/
	return phaseShift;
}

RtLbsType RayPath3D::GetAOD_Phi() const
{
	if (m_nodes.size() < 2)
		return 0.0;
	Vector3D pathStart = m_nodes[1]->m_point - m_nodes[0]->m_point;
	return pathStart.Azimuth();
}

RtLbsType RayPath3D::GetAOD_Theta() const
{
	if (m_nodes.size() < 2)
		return 0.0;
	Vector3D pathStart = m_nodes[1]->m_point - m_nodes[0]->m_point;
	return pathStart.Elevation();
}

RtLbsType RayPath3D::GetAOA_Phi() const
{
	if (m_nodes.size() < 2)
		return 0.0;
	Vector3D pathEnd = m_nodes[m_nodes.size() - 2]->m_point - m_nodes[m_nodes.size() - 1]->m_point;
	return pathEnd.Azimuth();
}

RtLbsType RayPath3D::GetAOA_Theta() const
{
	if (m_nodes.size() < 2)
		return 0.0;
	Vector3D pathEnd = m_nodes[m_nodes.size() - 2]->m_point - m_nodes[m_nodes.size() - 1]->m_point;
	return pathEnd.Elevation();
}

Point2D RayPath3D::GetGeneralSource2D() const
{
	Point2D gs = m_nodes[static_cast<int>(m_nodes.size()) - 2]->m_gs2D;
	return gs;
}

void RayPath3D::DeepCopy(const RayPath3D* path)
{
	if (path == nullptr)
		return;
	m_containRefract = path->m_containRefract;
	m_containSplitNode = path->m_containSplitNode;
	m_type = path->m_type;
	m_angularSpectrumCategoryId = path->m_angularSpectrumCategoryId;
	m_nodes.resize(path->m_nodes.size());
	for (int i = 0; i < path->m_nodes.size(); ++i) {
		m_nodes[i] = new PathNode3D(*path->m_nodes[i]);
	}
}

void RayPath3D::DeepDestroy()
{
	for (int i = 0; i < m_nodes.size(); ++i) {
		delete m_nodes[i];
	}
}

void RayPath3D::Clear()
{
	m_nodes.clear();
	m_containRefract = false;
	m_containSplitNode = false;
}

void RayPath3D::OutputRaypath(std::ofstream& stream) const
{
	stream << m_nodes.size() << " ";
	for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		const Point3D& p = (*it)->m_point;
		stream << p.x << " " << p.y << " " << p.z << " ";
	}
	stream << std::endl;
}

std::string RayPath3D::ToString() const
{
	std::stringstream ss;
	ss << m_nodes.size() << ",";
	for (auto& node : m_nodes) {
		ss << node->ToString();
	}
	return ss.str();
}

size_t RayPath3D::GetHash() const
{
	return util::Hash64(ToString());
}

