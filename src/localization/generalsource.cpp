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
	, m_fatherSource(nullptr)
	, m_replaceValidSource(nullptr)
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
	, m_fatherSource(s.m_fatherSource)
	, m_originPathNode(s.m_originPathNode)
	, m_replaceValidSource(s.m_replaceValidSource)
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
	m_fatherSource = s.m_fatherSource;
	m_originPathNode = s.m_originPathNode;
	m_replaceValidSource = s.m_replaceValidSource;
	return *this;
}

RtLbsType GeneralSource::CalculateSinglePathPower(const Point2D& p) const
{
	//构造路径
	std::vector<Point2D> positions(5);
	RtLbsType radius = 0.5;													/** @brief	活动半径设定为0.5m	*/
	positions[0] = Point2D(p[0], p[1]);					//原始坐标点
	positions[1] = Point2D(p[0] + radius, p[1]);		//up
	positions[2] = Point2D(p[0] - radius, p[1]);		//down
	positions[3] = Point2D(p[0], p[1] + radius);		//right
	positions[4] = Point2D(p[0], p[1] - radius);		//left

	//验证路径是否存在
	RayPath3D newPath;
	newPath.m_isValid = false;
	for (auto& curPos : positions) {
		if (HasValidReverseRayPath(curPos, newPath)) {
			break;
		}
	}
	//计算场功率
	RtLbsType receivedPower = -160;											//默认最低阈值为-160dBm
	if (newPath.m_isValid) {
		RtLbsType centerFrequency = _global_freqConfig.m_centerFrequency;	//获取中心频率
		const Antenna& omniAntenna = _global_innerAntennas[0];				//获取默认全向天线
		receivedPower = newPath.CalculatePowerInLBSSystem(centerFrequency, &omniAntenna);
	}
	//返回场功率
	return receivedPower;
}

bool GeneralSource::CalTDOAParameters_SPSTMD(const Point2D& targetPoint, const Scene* scene, RtLbsType freq, RtLbsType& delay, RtLbsType& power) const
{
	//根据Target计算路径,若在计算过程中路径不存在（反向射线追踪路径错误）
	std::vector<PathNode*> pathNodes;										/** @brief	构成路径的节点	*/
	pathNodes.push_back(new PathNode(targetPoint, NODE_ROOT));				/** @brief	纳入解作为其根节点	*/
	Point2D startPoint = targetPoint;										/** @brief	起始点坐标，用于验证路径是否有效	*/
	Segment2D testSegment(startPoint, m_position);							/** @brief	目标点和广义源构成的线段	*/
	Intersection2D testIntersection;										/** @brief	交点测试	*/
	if (m_type == NODE_REFL) {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//反射系列与环境相交，需要验证面元一致性
			if (testIntersection.m_segment->m_id != m_segment->m_id) {
				return false;
			}
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		pathNodes.back()->m_point = testIntersection.m_intersect;
		startPoint = testIntersection.m_intersect;
	}
	else {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//非反射系列与环境相交，表明路径无效,后续可能要增加透射机制，这里需要改进
			return false;
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		startPoint = m_position;
	}

	//进行迭代
	GeneralSource* curSource = m_fatherSource;
	while (curSource != nullptr) {
		testSegment = Segment2D(startPoint, curSource->m_position);
		if (curSource->m_type == NODE_REFL) {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//反射系列与环境相交，需要验证面元一致性
				if (testIntersection.m_segment->m_id != curSource->m_segment->m_id) {
					return false;
				}
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			pathNodes.back()->m_point = testIntersection.m_intersect;
			startPoint = testIntersection.m_intersect;
		}
		else {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//非反射系列与环境相交，表明路径无效,后续可能要增加透射机制，这里需要改进
				return false;
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			startPoint = curSource->m_position;
		}
		curSource = curSource->m_fatherSource;					//进行下一轮迭代
	}

	//根据nodes形成路径，并计算电磁				
	pathNodes.back()->m_type = NODE_STOP;						/** @brief	更改路径尾部属性	*/

	RayPath path2D(pathNodes, false);
	RayPath3D path3D(path2D, 2.0);
	Antenna* omniAntenna = scene->m_antennaLibrary.GetAntenna(0);					//获取全向天线
	power = path3D.CalculatePowerInLBSSystem(freq, omniAntenna);
	delay = path3D.GetPropagationTime();
	delete omniAntenna;
	return true;
}

bool GeneralSource::CalTDOAParameters_MPSTSD(const Point2D& targetPoint, const Scene* scene) const
{
	//根据Target计算路径,若在计算过程中路径不存在（反向射线追踪路径错误）
	std::vector<PathNode*> pathNodes;										/** @brief	构成路径的节点	*/
	pathNodes.push_back(new PathNode(targetPoint, NODE_ROOT));				/** @brief	纳入解作为其根节点	*/
	Point2D startPoint = targetPoint;										/** @brief	起始点坐标，用于验证路径是否有效	*/
	Segment2D testSegment(startPoint, m_position);							/** @brief	目标点和广义源构成的线段	*/
	Intersection2D testIntersection;										/** @brief	交点测试	*/
	if (m_type == NODE_REFL) {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//反射系列与环境相交，需要验证面元一致性
			if (testIntersection.m_segment->m_id != m_segment->m_id) {
				return false;
			}
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		pathNodes.back()->m_point = testIntersection.m_intersect;
		startPoint = testIntersection.m_intersect;
	}
	else {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//非反射系列与环境相交，表明路径无效,后续可能要增加透射机制，这里需要改进
			return false;
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		startPoint = m_position;
	}

	//进行迭代
	GeneralSource* curSource = m_fatherSource;
	while (curSource != nullptr) {
		testSegment = Segment2D(startPoint, curSource->m_position);
		if (curSource->m_type == NODE_REFL) {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//反射系列与环境相交，需要验证面元一致性
				if (testIntersection.m_segment->m_id != curSource->m_segment->m_id) {
					return false;
				}
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			pathNodes.back()->m_point = testIntersection.m_intersect;
			startPoint = testIntersection.m_intersect;
		}
		else {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//非反射系列与环境相交，表明路径无效,后续可能要增加透射机制，这里需要改进
				return false;
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			startPoint = curSource->m_position;
		}
		curSource = curSource->m_fatherSource;					//进行下一轮迭代
	}
	return true;			//路径真实存在
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

bool GeneralSource::HasValidReverseRayPath(Point2D& p, RayPath3D& outPath) const
{
	//根据Target计算路径,若在计算过程中路径不存在（反向射线追踪路径错误）
	std::vector<PathNode*> pathNodes;										/** @brief	构成路径的节点	*/
	pathNodes.push_back(new PathNode(p, NODE_ROOT));						/** @brief	纳入解作为其根节点	*/
	Point2D startPoint = p;													/** @brief	起始点坐标，用于验证路径是否有效	*/
	Segment2D testSegment(startPoint, m_position);							/** @brief	目标点和广义源构成的线段	*/
	Intersection2D testIntersection;										/** @brief	交点测试	*/
	if (m_type == NODE_REFL) {
		if (!m_segment->GetIntersect(testSegment, &testIntersection)) {
			for (auto& node : pathNodes) {delete node;}		//清除申请的内存
			return false;
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		pathNodes.back()->m_point = testIntersection.m_intersect;
		startPoint = testIntersection.m_intersect;
	}


	//寻找当前广义源的父广义源，并进行迭代
	GeneralSource* curSource = m_fatherSource;
	while (curSource != nullptr) {
		if (curSource->m_type == NODE_REFL) {
			testSegment = Segment2D(startPoint, curSource->m_position);
			if (!curSource->m_segment->GetIntersect(testSegment, &testIntersection)) {
				for (auto& node : pathNodes) { delete node; }		//清除申请的内存
				return false;
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			pathNodes.back()->m_point = testIntersection.m_intersect;
			startPoint = testIntersection.m_intersect;
		}
		else {
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			pathNodes.back()->m_point = curSource->m_position;
			startPoint = curSource->m_position;
		}
		curSource = curSource->m_fatherSource;					//进行下一轮迭代
	}
	//根据nodes形成路径，并计算电磁				
	pathNodes.back()->m_type = NODE_STOP;						/** @brief	更改路径尾部属性	*/
	RayPath path2D(pathNodes, false);
	outPath.Init(path2D, 2.0);
	outPath.m_isValid = true;									//路径设定为有效值
	return true;
}
