#include "gspair.h"
#include "gspaircluster.h"

GSPair::GSPair()
	: m_isValid(true)
	, m_clusterId(-1)
	, m_clusterSize(1)
	, m_gs1(nullptr)
	, m_gs2(nullptr)
	, m_gsRef(nullptr)
	, m_belongingPairCluster(nullptr)
	, m_phiResidual(0.0)
	, m_timeResidual(0.0)
	, m_timeDiffResidual(0.0)
	, m_powerResidual(0.0)
	, m_powerDiffResidual(0.0)
	, m_angularSpreadResidual(0.0)
	, m_delaySpreadResidual(0.0)
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
	, m_belongingPairCluster(nullptr)
	, m_phiResidual(0.0)
	, m_timeResidual(0.0)
	, m_timeDiffResidual(0.0)
	, m_powerResidual(0.0)
	, m_powerDiffResidual(0.0)
	, m_angularSpreadResidual(0.0)
	, m_delaySpreadResidual(0.0)
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
	, m_belongingPairCluster(nullptr)
	, m_phiResidual(0.0)
	, m_timeResidual(0.0)
	, m_timeDiffResidual(0.0)
	, m_powerResidual(0.0)
	, m_powerDiffResidual(0.0)
	, m_angularSpreadResidual(0.0)
	, m_delaySpreadResidual(0.0)
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
	, m_belongingPairCluster(pair.m_belongingPairCluster)
	, m_phiResidual(pair.m_phiResidual)
	, m_timeResidual(pair.m_timeResidual)
	, m_timeDiffResidual(pair.m_timeDiffResidual)
	, m_powerResidual(pair.m_powerResidual)
	, m_powerDiffResidual(pair.m_powerDiffResidual)
	, m_angularSpreadResidual(pair.m_angularSpreadResidual)
	, m_delaySpreadResidual(pair.m_delaySpreadResidual)
	, m_weight(pair.m_weight)
	, m_nullDataNum(pair.m_nullDataNum)
{
}

GSPair::~GSPair()
{
	//delete m_gs1;
	//delete m_gs2;
}

GSPair& GSPair::operator=(const GSPair& pair)
{
	m_isValid = pair.m_isValid;
	m_clusterId = pair.m_clusterId;
	m_clusterSize = pair.m_clusterSize;
	m_gs1=pair.m_gs1;
	m_gs2=pair.m_gs2;
	m_gsRef = pair.m_gsRef;
	m_belongingPairCluster = pair.m_belongingPairCluster;
	m_phiResidual = pair.m_phiResidual;
	m_timeResidual = pair.m_timeResidual;
	m_timeDiffResidual = pair.m_timeDiffResidual;
	m_powerResidual = pair.m_powerResidual;
	m_powerDiffResidual = pair.m_powerDiffResidual;
	m_angularSpreadResidual = pair.m_angularSpreadResidual;
	m_delaySpreadResidual = pair.m_delaySpreadResidual;
	m_weight = pair.m_weight;
	m_nullDataNum = pair.m_nullDataNum;
	return *this;
}

RtLbsType GSPair::DistanceTo(const GSPair& pair)
{
	return (m_targetSolution - pair.m_targetSolution).Length();
}

void GSPair::UpdateResidual_AOA(RtLbsType mean_r_phi, RtLbsType mean_r_powerDiff)
{
	m_phiResidual += m_nullDataNum * mean_r_phi;
	m_powerDiffResidual += m_nullDataNum * mean_r_powerDiff;
}

void GSPair::UpdateResidual_TOA(RtLbsType mean_r_time, RtLbsType mean_r_power)
{
	m_timeResidual += m_nullDataNum * mean_r_time;
	m_powerResidual += m_nullDataNum * mean_r_power;
}

void GSPair::UpdateResidual_TDOA(RtLbsType mean_r_timeDiff, RtLbsType mean_r_powerDiff)
{
	m_timeDiffResidual+=m_nullDataNum*mean_r_timeDiff;
	m_powerDiffResidual+=m_nullDataNum*mean_r_powerDiff;
}

void GSPair::UpdateResidual_AOATOA(RtLbsType mean_r_phi, RtLbsType mean_r_time, RtLbsType mean_r_power)
{
	m_phiResidual += m_nullDataNum * mean_r_phi;
	m_timeResidual += m_nullDataNum * mean_r_time;
	m_powerResidual += m_nullDataNum * mean_r_power;
}

void GSPair::UpdateResidual_AOATDOA(RtLbsType mean_r_phi, RtLbsType mean_r_timeDiff, RtLbsType mean_r_powerDiff)
{
	m_phiResidual += m_nullDataNum * mean_r_phi;
	m_timeDiffResidual += m_nullDataNum * mean_r_timeDiff;
	m_powerDiffResidual += m_nullDataNum * mean_r_powerDiff;
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

	if (!_judgementRules(scene)) {
		return false;
	}

	return true;
}

bool GSPair::HasValidTOASolution(const Scene* scene)
{
	//���ж���������Դ�Ƿ�������һ������������
	if (m_gs1->m_sensorData.m_id == m_gs2->m_sensorData.m_id) {				//����������Դ��������ͬ����Ϊ��Ч����Դ
		m_isValid = false;
		return false;
	}
	if (!_calTOASolution(scene->m_bbox)) {
		m_isValid = false;
		return false;
	}

	if (!_judgementRules(scene)) {
		return false;
	}


	return true;
}

bool GSPair::HasValidAOATOASolution(const Scene* scene)
{
	//���ж���������Դ�Ƿ�������һ������������
	if (m_gs1->m_sensorData.m_id == m_gs2->m_sensorData.m_id) {				//����������Դ��������ͬ����Ϊ��Ч����Դ
		m_isValid = false;
		return false;
	}

	//�ȼ���AOA��ֵ�⣬�ڴ���TOA���н���
	if (!_calAOASolution()) {
		m_isValid = false;
		return false;
	}

	if (!_calAOATOASolution(scene->m_bbox)) {			//���������н⣬�Ž�����һ��
		m_isValid = false;
		return false;
	}

	if (!_judgementRules(scene)) {
		return false;
	}

	return true;

}

bool GSPair::HasValidAOATDOASolution(const Scene* scene)
{
	//���ж���������Դ�Ƿ�������һ������������
	if (m_gs1->m_sensorData.m_id == m_gs2->m_sensorData.m_id) {				//����������Դ��������ͬ����Ϊ��Ч����Դ
		m_isValid = false;
		return false;
	}

	//�ȼ���AOA��ֵ�⣬�ڴ���TOA���н���
	if (!_calAOASolution()) {
		m_isValid = false;
		return false;
	}

	if (!_calAOATDOASolution(scene->m_bbox)) {			//���������н⣬�Ž�����һ��
		m_isValid = false;
		return false;
	}

	if (!_judgementRules(scene)) {
		return false;
	}

	return true;
}

bool GSPair::HasValidTDOASolution(const Scene* scene)
{
	if (!_calTDOASolution(scene->m_bbox)) {
		return false;
	}

	if (!_judgementRules(scene)) {
		return false;
	}
	return true;
}

void GSPair::CalculateSinglePairResidual()
{
	//1-׷����Դ

	//2-�����ų�

	//3-����в�
}

void GSPair::CalNormalizedWeightAndUpdate_AOA(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum)
{
	RtLbsType r_norm_phi = m_phiResidual / max_r_phi;												/** @brief	��һ���ĽǶȲв�	*/
	RtLbsType r_norm_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	if (r_norm_phi < 1e-4) { r_norm_phi = 1e-4; }
	if (r_norm_powerDiff < 1e-4) { r_norm_powerDiff = 1e-4; }
	RtLbsType w_phi = 1.0 / r_norm_phi;													/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / r_norm_powerDiff;										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = static_cast<RtLbsType>(m_clusterSize) / static_cast<RtLbsType>(max_clusterNum);		/** @brief	����Ȩ��	*/
	RtLbsType w_total = w.m_phiWeight * w_phi + w.m_powerWeight * w_powerDiff;							/** @brief	��Ȩ��	*/
	if (m_belongingPairCluster->m_isDeviateSolution) {																				//��pair������clusterΪƫ���(��Ч��)����Ȩ������
		w_total = 0.0;	
	}
	m_gs1->m_weight = std::max(m_gs1->m_weight, w_total);
	m_gs2->m_weight = std::max(m_gs2->m_weight, w_total);
	m_weight = w_total;
	m_belongingPairCluster->m_weight = m_weight;												//��ֵ��������Ȩ��
}

void GSPair::CalNormalizedWeightAndUpdate_TOA(RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w, int max_clusterNum)
{
	RtLbsType r_norm_time = m_timeResidual / max_r_time;														/** @brief	��һ����ʱ��в�	*/
	RtLbsType r_norm_power = m_powerResidual / max_r_power;														/** @brief	��һ���Ĺ��ʲв�	*/
	if (r_norm_time < 1e-4) { r_norm_time = 1e-4; }
	if (r_norm_power < 1e-4) { r_norm_power = 1e-4; }
	RtLbsType w_time = 1.0 / r_norm_time;																		/** @brief	ʱ��Ȩ��	*/
	RtLbsType w_power = 1.0 / r_norm_power;																		/** @brief	����Ȩ��	*/
	RtLbsType w_cluster = static_cast<RtLbsType>(m_clusterSize) / static_cast<RtLbsType>(max_clusterNum);		/** @brief	����Ȩ��	*/
	RtLbsType w_total = w.m_timeWeight * w_time + w.m_powerWeight * w_power;
	if (m_belongingPairCluster->m_isDeviateSolution) { w_total = 0; }
	m_gs1->m_weight = std::max(m_gs1->m_weight, w_total);
	m_gs2->m_weight = std::max(m_gs2->m_weight, w_total);
	m_weight = w_total;
	m_belongingPairCluster->m_weight = m_weight;																//��ֵ��������Ȩ��
}

void GSPair::CalNormalizedWeightAndUpdate_AOATOA(RtLbsType max_r_phi, RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w, int max_clusterNum)
{
	RtLbsType r_norm_phi = m_phiResidual / max_r_phi;															/** @brief	��һ���ĽǶȲв�	*/
	RtLbsType r_norm_time = m_timeResidual / max_r_time;														/** @brief	��һ����ʱ��в�	*/
	RtLbsType r_norm_power = m_powerResidual / max_r_power;														/** @brief	��һ���Ĺ��ʲв�	*/
	if (r_norm_phi < 1e-4) { r_norm_phi = 1e-4; }
	if (r_norm_time < 1e-4) { r_norm_time = 1e-4; }
	if (r_norm_power < 1e-4) { r_norm_power = 1e-4; }
	RtLbsType w_phi = 1.0 / r_norm_phi;																			/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType w_time = 1.0 / r_norm_time;																		/** @brief	ʱ��Ȩ��	*/
	RtLbsType w_power = 1.0 / r_norm_power;																		/** @brief	����Ȩ��	*/
	RtLbsType w_cluster = static_cast<RtLbsType>(m_clusterSize) / static_cast<RtLbsType>(max_clusterNum);		/** @brief	����Ȩ��	*/
	RtLbsType w_total = w.m_phiWeight * w_phi + w.m_timeWeight * w_time + w.m_powerWeight * w_power;
	if (m_belongingPairCluster->m_isDeviateSolution) { w_total = 0; }
	m_gs1->m_weight = std::max(m_gs1->m_weight, w_total);
	m_gs2->m_weight = std::max(m_gs2->m_weight, w_total);
	m_weight = w_total;
	m_belongingPairCluster->m_weight = m_weight;																//��ֵ��������Ȩ��
}

void GSPair::CalNormalizedWeightAndUpdate_TDOA(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum)
{
	if (m_timeDiffResidual == 0) {
		std::cout << "find it" << std::endl;
	}
	RtLbsType r_normalized_timeDiff = m_timeDiffResidual / max_r_timeDiff;									/** @brief	��һ����ʱ���в�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-6);										/** @brief	ʱ���Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-6);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	����Ȩ��	*/
	RtLbsType w_total = w.m_timeWeight * w_timeDiff + w.m_powerWeight * w_powerDiff;		/** @brief	��Ȩ��	*/
	if (m_belongingPairCluster->m_isDeviateSolution) { w_total = 0; }
	if (m_gs1->m_weight < w_total) {
		m_gs1->m_weight = w_total;
	}
	if (m_gs2->m_weight < w_total) {
		m_gs2->m_weight = w_total;
	}
	m_weight = w_total;
	if (m_belongingPairCluster->m_isValid == false) {			//����������Ϊ��Ч�أ���Ȩ������
		m_belongingPairCluster->m_weight = 0;
	}
	if(m_belongingPairCluster->m_weight < m_weight)				//������������Ȩ��
		m_belongingPairCluster->m_weight = m_weight;														//��ֵ��������Ȩ��
}

void GSPair::CalNormalizedWeightAndUpdate_AOATDOA(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum)
{
	RtLbsType r_normalized_phi = m_phiResidual / max_r_phi;												/** @brief	��һ���ĽǶȲв�	*/
	RtLbsType r_normalized_timeDiff = m_timeDiffResidual / max_r_timeDiff;									/** @brief	��һ����ʱ���в�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_phi = 1.0 / (r_normalized_phi + 1e-4);													/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-4);										/** @brief	ʱ���Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-4);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	����Ȩ��	*/
	RtLbsType w_total = w.m_phiWeight * w_phi + w.m_timeWeight * w_timeDiff + w.m_powerWeight * w_powerDiff;		/** @brief	��Ȩ��	*/
	if (m_belongingPairCluster->m_isDeviateSolution) { w_total = 0; }
	if (m_gs1->m_weight < w_total) {
		m_gs1->m_weight = w_total;
	}
	if (m_gs2->m_weight < w_total) {
		m_gs2->m_weight = w_total;
	}
	m_weight = w_total;
	if (m_belongingPairCluster->m_weight < m_weight) {
		m_belongingPairCluster->m_weight = m_weight;																//��ֵ��������Ȩ��
	}
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

bool GSPair::_calTOASolution(const BBox2D& bbox)
{
	//����ceres���TOA��
	TOASolver solver;
	solver.SetGeneralSource(m_gs1, m_gs2);
	RtLbsType accuracy = solver.Solving_LS(bbox, m_targetSolution);
	if (accuracy > EPSILON) {													//�����ȸ���EPSILON���϶�Ϊ���鷽��Ϊ�޽����
		return false;
	}
	return true;
}

bool GSPair::_calTDOASolution(const BBox2D& bbox)
{
	//����ceres���TDOA��
	TDOASolver tdoaSolver;
	tdoaSolver.SetGeneralSource(m_gsRef, m_gs1, m_gs2);				//�趨����Դ
	RtLbsType accuracy = tdoaSolver.Solving_LS(bbox, m_targetSolution);
	if (accuracy > EPSILON) {													//�����ȸ���EPSILON���϶�Ϊ���鷽��Ϊ�޽����
		return false;
	}
	return true;
}

bool GSPair::_calAOATOASolution(const BBox2D& bbox)
{
	//����ceres���AOA-TOA��
	AOATOASolver solver;
	solver.SetGeneralSource(m_gs1, m_gs2);
	RtLbsType accuracy = solver.Solving_LS(bbox, m_targetSolution);
	if (accuracy > 1) {													//�����ȸ���EPSILON���϶�Ϊ���鷽��Ϊ�޽����
		return false;
	}
	return true;
}

bool GSPair::_calAOATDOASolution(const BBox2D& bbox)
{
	//����ceres���AOA-TOA��
	AOATDOASolver solver;
	solver.SetGeneralSource(m_gsRef, m_gs1, m_gs2);
	RtLbsType accuracy = solver.Solving_LS(bbox, m_targetSolution);
	if (accuracy > 1) {
		return false;
	}
	return true;
}

bool GSPair::_judgementRules(const Scene* scene)
{
	//�ж�׼��0-���Ƿ�ʹ�����λ���ظ�
	for (auto& curSensor : scene->m_sensors) {
		Point2D curSensorPoint = curSensor->GetPosition2D();
		if (Distance(curSensorPoint, m_targetSolution) < 1e-1) {			//��������봫����λ��С��10cm�����������Ч
			return false;
		}
	}

	//�ж�׼��1- �Ƿ��ڻ�������Чλ��(���������߽���ڽ����ڲ�)
	if (!scene->IsValidPoint(m_targetSolution)) {
		//������չһ�����룬�ж��Ƿ��뻷���ཻ
		std::vector<Point2D> extendTargetSolution(4);
		extendTargetSolution[0] = m_targetSolution + Point2D(0, 0.1);
		extendTargetSolution[1] = m_targetSolution + Point2D(0, -0.1);
		extendTargetSolution[2] = m_targetSolution + Point2D(0.1, 0);
		extendTargetSolution[3] = m_targetSolution + Point2D(-0.1, 0);
		bool hasValidSolution = true;
		for (auto& curSolution : extendTargetSolution) {
			if (!scene->IsValidPoint(curSolution)) {
				hasValidSolution = false;
			}
		}
		if (!hasValidSolution) {
			m_isValid = false;
			return false;
		}
	}

	//�ж�׼��2-�����Ƿ��ر���ǽ���Ե��һ�㶨��0.5m��Ϊǽ���Ե����
	if (scene->IsNearSegmentPoint(m_targetSolution, 0.1)) {
		m_isValid = false;
		return false;
	}

	//�ж�׼��3- ������AOA����������£�����Դ���ڴ����ڵ���Ŀ���Ĺ��ɵ�ĩ��·���Ƿ񱻻������ڵ�
	const Point2D& np1 = m_gs1->m_position;							/** @brief	����Դ1���ڽڵ�����	*/
	const Point2D& np2 = m_gs2->m_position;							/** @brief	����Դ2���ڽڵ�����	*/

	Segment2D segment1(m_targetSolution, np1);							/** @brief	��ϲ����߶�1	*/
	if (m_gs1->m_type == NODE_REFL) {
		Intersection2D testIntersect;
		if (!scene->GetIntersect(segment1, &testIntersect)) {						//���߶�1�뻷�����ཻ�����������Ч
			m_isValid = false;
			return false;
		}
		if (testIntersect.m_segment->m_id != m_gs1->m_segment->m_id) {				//�������߶���ԭʼ��������Դ���߶β�һ�£������Ч
			m_isValid = false;
			return false;
		}
	}
	else if (m_gs1->m_type == NODE_ROOT || m_gs1->m_type == NODE_DIFF) {
		if (scene->GetIntersect(segment1, nullptr)) {								//�����ڵ������������߶�1�뻷���ཻ�����������Ч
			m_isValid = false;
			return false;
		}
	}

	Segment2D segment2(m_targetSolution, np2);							/** @brief	��ϲ����߶�2	*/
	if (m_gs2->m_type == NODE_REFL) {
		Intersection2D testIntersect;
		if (!scene->GetIntersect(segment2, &testIntersect)) {						//���߶�1�뻷�����ཻ�����������Ч
			m_isValid = false;
			return false;
		}
		if (testIntersect.m_segment->m_id != m_gs2->m_segment->m_id) {				//�������߶���ԭʼ��������Դ���߶β�һ�£������Ч
			m_isValid = false;
			return false;
		}
	}
	else if (m_gs2->m_type == NODE_ROOT || m_gs2->m_type == NODE_DIFF) {
		if (scene->GetIntersect(segment2, nullptr)) {								//�����ڵ������������߶�1�뻷���ཻ�����������Ч
			m_isValid = false;
			return false;
		}
	}
	return true;
}
