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

bool GeneralSource::CalTDOAParameters_SPSTMD(const Point2D& targetPoint, const Scene* scene, RtLbsType freq, const std::vector<Complex>& tranFunction, RtLbsType& delay, RtLbsType& power) const
{
	//����Target����·��,���ڼ��������·�������ڣ���������׷��·������
	std::vector<PathNode*> pathNodes;										/** @brief	����·���Ľڵ�	*/
	pathNodes.push_back(new PathNode(targetPoint, NODE_ROOT));				/** @brief	�������Ϊ����ڵ�	*/
	Point2D startPoint = targetPoint;										/** @brief	��ʼ�����꣬������֤·���Ƿ���Ч	*/
	Segment2D testSegment(startPoint, m_position);							/** @brief	Ŀ���͹���Դ���ɵ��߶�	*/
	Intersection2D testIntersection;										/** @brief	�������	*/
	if (m_type == NODE_REFL) {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//����ϵ���뻷���ཻ����Ҫ��֤��Ԫһ����
			if (testIntersection.m_segment->m_id != m_segment->m_id) {
				return false;
			}
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		pathNodes.back()->m_point = testIntersection.m_intersect;
		startPoint = testIntersection.m_intersect;
	}
	else {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//�Ƿ���ϵ���뻷���ཻ������·����Ч,��������Ҫ����͸����ƣ�������Ҫ�Ľ�
			return false;
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		startPoint = m_position;
	}

	//���е���
	GeneralSource* curSource = m_fatherSource;
	while (curSource != nullptr) {
		testSegment = Segment2D(startPoint, curSource->m_position);
		if (curSource->m_type == NODE_REFL) {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//����ϵ���뻷���ཻ����Ҫ��֤��Ԫһ����
				if (testIntersection.m_segment->m_id != curSource->m_segment->m_id) {
					return false;
				}
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			pathNodes.back()->m_point = testIntersection.m_intersect;
			startPoint = testIntersection.m_intersect;
		}
		else {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//�Ƿ���ϵ���뻷���ཻ������·����Ч,��������Ҫ����͸����ƣ�������Ҫ�Ľ�
				return false;
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			startPoint = curSource->m_position;
		}
		curSource = curSource->m_fatherSource;					//������һ�ֵ���
	}

	//����nodes�γ�·������������				
	pathNodes.back()->m_type = NODE_STOP;						/** @brief	����·��β������	*/

	RayPath path2D(pathNodes, false);
	RayPath3D path3D(path2D, 2.0);
	Antenna* omniAntenna = scene->m_antennaLibrary.GetAntenna(0);					//��ȡȫ������
	power = path3D.CalculatePowerInLBSSystem(freq, tranFunction, &scene->m_materialLibrary, omniAntenna);
	delay = path3D.GetPropagationTime();
	return true;
}

bool GeneralSource::CalTDOAParameters_MPSTSD(const Point2D& targetPoint, const Scene* scene) const
{
	//����Target����·��,���ڼ��������·�������ڣ���������׷��·������
	std::vector<PathNode*> pathNodes;										/** @brief	����·���Ľڵ�	*/
	pathNodes.push_back(new PathNode(targetPoint, NODE_ROOT));				/** @brief	�������Ϊ����ڵ�	*/
	Point2D startPoint = targetPoint;										/** @brief	��ʼ�����꣬������֤·���Ƿ���Ч	*/
	Segment2D testSegment(startPoint, m_position);							/** @brief	Ŀ���͹���Դ���ɵ��߶�	*/
	Intersection2D testIntersection;										/** @brief	�������	*/
	if (m_type == NODE_REFL) {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//����ϵ���뻷���ཻ����Ҫ��֤��Ԫһ����
			if (testIntersection.m_segment->m_id != m_segment->m_id) {
				return false;
			}
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		pathNodes.back()->m_point = testIntersection.m_intersect;
		startPoint = testIntersection.m_intersect;
	}
	else {
		if (scene->GetIntersect(testSegment, &testIntersection)) {				//�Ƿ���ϵ���뻷���ཻ������·����Ч,��������Ҫ����͸����ƣ�������Ҫ�Ľ�
			return false;
		}
		pathNodes.push_back(new PathNode(m_originPathNode));
		startPoint = m_position;
	}

	//���е���
	GeneralSource* curSource = m_fatherSource;
	while (curSource != nullptr) {
		testSegment = Segment2D(startPoint, curSource->m_position);
		if (curSource->m_type == NODE_REFL) {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//����ϵ���뻷���ཻ����Ҫ��֤��Ԫһ����
				if (testIntersection.m_segment->m_id != curSource->m_segment->m_id) {
					return false;
				}
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			pathNodes.back()->m_point = testIntersection.m_intersect;
			startPoint = testIntersection.m_intersect;
		}
		else {
			if (scene->GetIntersect(testSegment, &testIntersection)) {				//�Ƿ���ϵ���뻷���ཻ������·����Ч,��������Ҫ����͸����ƣ�������Ҫ�Ľ�
				return false;
			}
			pathNodes.push_back(new PathNode(curSource->m_originPathNode));
			startPoint = curSource->m_position;
		}
		curSource = curSource->m_fatherSource;					//������һ�ֵ���
	}
	return true;			//·����ʵ����
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
