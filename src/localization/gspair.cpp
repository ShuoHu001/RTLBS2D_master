#include "gspair.h"

GSPair::GSPair()
	: m_isValid(true)
	, m_clusterId(-1)
	, m_clusterSize(1)
	, m_gs1(nullptr)
	, m_gs2(nullptr)
	, m_gsRef(nullptr)
	, m_phiResidual(0.0)
	, m_timeResidual(0.0)
	, m_timeDiffResidual(0.0)
	, m_powerDiffResidual(0.0)
	, m_weight(0.0)
	, m_nullDataNum(0)
{
}

GSPair::GSPair(GeneralSource* gs1, GeneralSource* gs2)
	: m_isValid(true)
	, m_clusterId(-1)
	, m_clusterSize(1)
	, m_gs1(gs1)
	, m_gs2(gs2)
	, m_gsRef(nullptr)
	, m_phiResidual(0.0)
	, m_timeResidual(0.0)
	, m_timeDiffResidual(0.0)
	, m_powerDiffResidual(0.0)
	, m_weight(0.0)
	, m_nullDataNum(0)
{
}

GSPair::GSPair(GeneralSource* gsRef, GeneralSource* gs1, GeneralSource* gs2)
	: m_isValid(true)
	, m_clusterId(-1)
	, m_clusterSize(1)
	, m_gs1(gs1)
	, m_gs2(gs2)
	, m_gsRef(gsRef)
	, m_phiResidual(0.0)
	, m_timeResidual(0.0)
	, m_timeDiffResidual(0.0)
	, m_powerDiffResidual(0.0)
	, m_weight(0.0)
	 ,m_nullDataNum(0)
{
}

GSPair::GSPair(const GSPair& pair)
	: m_isValid(pair.m_isValid)
	, m_clusterId(pair.m_clusterId)
	, m_clusterSize(pair.m_clusterSize)
	, m_gs1(pair.m_gs1)
	, m_gs2(pair.m_gs2)
	, m_gsRef(pair.m_gsRef)
	, m_phiResidual(pair.m_phiResidual)
	, m_timeResidual(pair.m_timeResidual)
	, m_timeDiffResidual(pair.m_timeDiffResidual)
	, m_powerDiffResidual(pair.m_powerDiffResidual)
	, m_weight(pair.m_weight)
	, m_nullDataNum(pair.m_nullDataNum)
{
}

GSPair::~GSPair()
{
}

GSPair& GSPair::operator=(const GSPair& pair)
{
	m_isValid = pair.m_isValid;
	m_clusterId = pair.m_clusterId;
	m_clusterSize = pair.m_clusterSize;
	m_gs1=pair.m_gs1;
	m_gs2=pair.m_gs2;
	m_gsRef = pair.m_gsRef;
	m_phiResidual = pair.m_phiResidual;
	m_timeResidual = pair.m_timeResidual;
	m_timeDiffResidual = pair.m_timeDiffResidual;
	m_powerDiffResidual = pair.m_powerDiffResidual;
	m_weight = pair.m_weight;
	m_nullDataNum = pair.m_nullDataNum;
	return *this;
}

RtLbsType GSPair::DistanceTo(const GSPair& pair)
{
	return (m_targetSolution - pair.m_targetSolution).Length();
}

void GSPair::NormalizedWeight(RtLbsType max_weight)
{
	m_weight = max_weight / max_weight;
}

bool GSPair::HasValidAOASolution(const Scene* scene)
{
	if (!_calAOASolution()) {											//������ЧAOA�⣬�򷵻�false
		m_isValid = false;
		return false;
	}

	//�ж�׼��0-���Ƿ�ʹ�����λ���ظ�
	for (auto curSensor : scene->m_sensors) {
		Point2D curSensorPoint = curSensor->GetPosition2D();
		if (Distance(curSensorPoint, m_targetSolution) < 1e-1) {			//��������봫����λ��С��10cm�����������Ч
			return false;
		}
	}


	//�ж�׼��1- �Ƿ��ڻ�������Чλ��(���������߽���ڽ����ڲ�)
	if (!scene->IsValidPoint(m_targetSolution)) {
		m_isValid = false;
		return false;
	}

	//�ж�׼��2- ������AOA����������£�����Դ���ڴ����ڵ���Ŀ���Ĺ��ɵ�ĩ��·���Ƿ񱻻������ڵ�
	const Point2D& np1 = m_gs1->m_nodePosition;							/** @brief	����Դ1���ڽڵ�����	*/
	const Point2D& np2 = m_gs2->m_nodePosition;							/** @brief	����Դ2���ڽڵ�����	*/

	Segment2D segment1(m_targetSolution, np1);							/** @brief	��ϲ����߶�1	*/
	Segment2D segment2(m_targetSolution, np2);							/** @brief	��ϲ����߶�2	*/

	if (scene->GetIntersect(segment1, nullptr)) {						//���߶�1�뻷���ཻ�����������Ч
		m_isValid = false;
		return false;
	}
	if (scene->GetIntersect(segment2, nullptr)) {						//���߶�2�뻷���ཻ�����������Ч
		m_isValid = false;
		return false;
	}

	//���е��˴�֤����Ϊ��Ч�⣬Ȩ�ؼ���+1
	m_gs1->m_wCount += 1;
	m_gs2->m_wCount += 1;

	return true;
}

bool GSPair::HasValidTOASolution(const Scene* scene)
{
	return false;
}

bool GSPair::HasValidTDOASolution(const Scene* scene)
{
	if (!_calTDOASolution()) {
		m_isValid = false;
		return false;
	}

	//�ж�׼��0-���Ƿ�ʹ�����λ���ظ�
	for (auto curSensor : scene->m_sensors) {
		Point2D curSensorPoint = curSensor->GetPosition2D();
		if (Distance(curSensorPoint, m_targetSolution) < 1e-1) {			//��������봫����λ��С��10cm�����������Ч
			return false;
		}
	}


	//�ж�׼��1- �Ƿ��ڻ�������Чλ��(���������߽���ڽ����ڲ�)
	if (!scene->IsValidPoint(m_targetSolution)) {
		m_isValid = false;
		return false;
	}

	//�ж�׼��2- ������AOA����������£�����Դ���ڴ����ڵ���Ŀ���Ĺ��ɵ�ĩ��·���Ƿ񱻻������ڵ�
	const Point2D& np1 = m_gs1->m_position;								/** @brief	����Դ1����	*/
	const Point2D& np2 = m_gs2->m_position;								/** @brief	����Դ2����	*/

	Segment2D segment1(m_targetSolution, np1);							/** @brief	��ϲ����߶�1	*/
	Segment2D segment2(m_targetSolution, np2);							/** @brief	��ϲ����߶�2	*/

	if (m_gs1->m_type == NODE_REFL) {									//�������Դ����
		Intersection2D intersect;
		if (!scene->GetIntersect(segment1, &intersect)) {				//�߶��뷴���治�ཻ������⼯�����ɵ�·��������
			return false;
		}
		if (intersect.m_segment->m_id != m_gs1->m_segment->m_id) {		//���߶�1�뻷���ཻ���佻�����ڻ����߶����������Դ���߶β�ͬ����������Ч
			m_isValid = false;
			return false;
		}
	}
	else {																//���䡢���ڵ����Դ����
		Intersection2D intersect;
		if (scene->GetIntersect(segment1, &intersect)) {				//��Ŀ�����������䡢���ڵ����Դ�乹�ɵ��߶��뻷���ཻ�����������Ч
			m_isValid = false;
			return false;
		}
	}
	
	if (m_gs2->m_type == NODE_REFL) {									//�������Դ����
		Intersection2D intersect;
		if (!scene->GetIntersect(segment2, &intersect)) {				//�߶��뷴���治�ཻ������⼯�����ɵ�·��������
			return false;
		}
		if (intersect.m_segment->m_id != m_gs2->m_segment->m_id) {		//���߶�2�뻷���ཻ���佻�����ڻ����߶����������Դ���߶β�ͬ����������Ч
			m_isValid = false;
			return false;
		}
	}
	else {
		Intersection2D intersect;
		if (scene->GetIntersect(segment2, &intersect)) {				//��Ŀ�����������䡢���ڵ����Դ�乹�ɵ��߶��뻷���ཻ�����������Ч
			m_isValid = false;
			return false;
		}
	}
	
	//���е��˴�֤����Ϊ��Ч�⣬Ȩ�ؼ���+1
	m_gs1->m_wCount += 1;
	m_gs2->m_wCount += 1;

	return true;
}

void GSPair::CalNormalizedWeightAndUpdate_AOA(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, int max_clusterNum)
{
	RtLbsType r_normalized_phi = m_phiResidual / max_r_phi;												/** @brief	��һ���ĽǶȲв�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_phi = 1.0 / (r_normalized_phi + 1e-4);													/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-4);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	����Ȩ��	*/
	RtLbsType w_total = ONE_THIRD * w_phi + ONE_THIRD * w_powerDiff + ONE_THIRD * w_cluster;			/** @brief	��Ȩ��	*/
	if (m_gs1->m_weight < w_total) {
		m_gs1->m_weight = w_total;
	}
	if (m_gs2->m_weight < w_total) {
		m_gs2->m_weight = w_total;
	}
	m_weight = w_total;
}

void GSPair::CalNormalizedWeightAndUpdate_TDOA(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, int max_clusterNum)
{
	RtLbsType r_normalized_timeDiff = m_phiResidual / max_r_timeDiff;									/** @brief	��һ����ʱ���в�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-6);										/** @brief	ʱ���Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-6);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	����Ȩ��	*/
	RtLbsType w_total = ONE_THIRD * w_timeDiff + ONE_THIRD * w_powerDiff + +ONE_THIRD * w_cluster;		/** @brief	��Ȩ��	*/
	if (m_gs1->m_weight < w_total) {
		m_gs1->m_weight = w_total;
	}
	if (m_gs2->m_weight < w_total) {
		m_gs2->m_weight = w_total;
	}
	m_weight = w_total;
}

void GSPair::CalNormalizedWeightAndUpdate_AOA_TDOA(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, int max_clusterNum)
{
	RtLbsType r_normalized_phi = m_phiResidual / max_r_phi;												/** @brief	��һ���ĽǶȲв�	*/
	RtLbsType r_normalized_timeDiff = m_phiResidual / max_r_timeDiff;									/** @brief	��һ����ʱ���в�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_phi = 1.0 / (r_normalized_phi + 1e-6);													/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-6);										/** @brief	ʱ���Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-6);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	����Ȩ��	*/
	RtLbsType w_total = 0.25 * w_phi + 0.25 * w_timeDiff + 0.25 * w_powerDiff + 0.25 * w_cluster;		/** @brief	��Ȩ��	*/
	if (m_gs1->m_weight < w_total) {
		m_gs1->m_weight = w_total;
	}
	if (m_gs2->m_weight < w_total) {
		m_gs2->m_weight = w_total;
	}
	m_weight = w_total;
}

bool GSPair::_calAOASolution()
{
	RtLbsType theta1 = m_gs1->m_sensorData.m_phi;
	RtLbsType theta2 = m_gs2->m_sensorData.m_phi;
	RtLbsType x1 = m_gs1->m_position.x;
	RtLbsType x2 = m_gs2->m_position.x;
	RtLbsType y1 = m_gs1->m_position.y;
	RtLbsType y2 = m_gs2->m_position.y;

	if (std::abs(theta1 - theta2) < EPSILON) {			//���Ƕ���ȣ������н�
		return false;
	}

	RtLbsType sinTheta1 = sin(theta1);
	RtLbsType cosTheta1 = cos(theta1);
	RtLbsType sinTheta2 = sin(theta2);
	RtLbsType cosTheta2 = cos(theta2);
	RtLbsType sinTheta12 = sin(theta1 - theta2);

	RtLbsType x = ((x1 * sinTheta1 - y1 * cosTheta1) * cosTheta2 - (x2 * sinTheta2 - y2 * cosTheta2) * cosTheta1) / sinTheta12;
	RtLbsType y = ((x1 * sinTheta1 - y1 * cosTheta1) * sinTheta2 - (x2 * sinTheta2 - y2 * cosTheta2) * sinTheta1) / sinTheta12;

	m_targetSolution.x = x;
	m_targetSolution.y = y;

	return true;
}

bool GSPair::_calTOASolution()
{
	return false;
}

bool GSPair::_calTDOASolution()
{
	//����ceres���TDOA��
	TDOASolver tdoaSolver;
	tdoaSolver.SetGeneralSource(m_gsRef, m_gs1, m_gs2);				//�趨����Դ
	RtLbsType accuracy = tdoaSolver.Solving_LS(m_targetSolution);
	if (accuracy > EPSILON) {													//�����ȸ���EPSILON���϶�Ϊ���鷽��Ϊ�޽����
		return false;
	}
	return true;
}
