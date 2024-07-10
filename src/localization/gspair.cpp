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
	, m_gs1(new GeneralSource(*gs1))
	, m_gs2(new GeneralSource(*gs2))
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
	, m_gs1(new GeneralSource(*gs1))
	, m_gs2(new GeneralSource(*gs2))
	, m_gsRef(new GeneralSource(*gsRef))
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
	delete m_gs1;
	delete m_gs2;
	if (m_gsRef != nullptr) {
		delete m_gsRef;
	}
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
	for (auto& curSensor : scene->m_sensors) {
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

	//�ж�׼��2-�����Ƿ��ر���ǽ���Ե��һ�㶨��0.5m��Ϊǽ���Ե����
	if (scene->IsNearSegmentPoint(m_targetSolution, 0.5)) {
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
	else if(m_gs2->m_type == NODE_ROOT || m_gs2->m_type == NODE_DIFF) {
		if (scene->GetIntersect(segment2, nullptr)) {								//�����ڵ������������߶�1�뻷���ཻ�����������Ч
			m_isValid = false;
			return false;
		}
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

bool GSPair::HasValidTDOASolution_SPSTMD(const Scene* scene, RtLbsType freq)
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

	//�ж�׼��2- ���շ�������׷�ټ���ģʽ�������Դ����������Դ֮���·���Ƿ���ʵ����

	RtLbsType delay_refSource = 0.0;
	RtLbsType power_refSource = 0.0;
	if (!m_gsRef->CalTDOAParameters_SPSTMD(m_targetSolution, scene, freq, delay_refSource, power_refSource)) {
		return false;
	}

	RtLbsType delay_source1 = 0.0;
	RtLbsType power_source1 = 0.0;
	if (!m_gs1->CalTDOAParameters_SPSTMD(m_targetSolution, scene, freq, delay_source1, power_source1)) {
		return false;
	}

	RtLbsType delay_source2 = 0.0;
	RtLbsType power_source2 = 0.0;
	if (!m_gs2->CalTDOAParameters_SPSTMD(m_targetSolution, scene, freq, delay_source2, power_source2)) {
		return false;
	}
	
	RtLbsType timeDiff_gs1 = (delay_source1 - delay_refSource) * 1e9;
	RtLbsType timeDiff_gs2 = (delay_source2 - delay_refSource) * 1e9;
	RtLbsType powerDiff_gs1 = power_source1 - power_refSource;
	RtLbsType powerDiff_gs2 = power_source2 - power_refSource;

	RtLbsType r_timeDiff_gs1 = timeDiff_gs1 - m_gs1->m_sensorData.m_timeDiff * 1e9;						//ת��Ϊns
	RtLbsType r_timeDiff_gs2 = timeDiff_gs2 - m_gs2->m_sensorData.m_timeDiff * 1e9;
	RtLbsType r_powerDiff_gs1 = powerDiff_gs1 - (m_gs1->m_sensorData.m_power - m_gsRef->m_sensorData.m_power);
	RtLbsType r_powerDiff_gs2 = powerDiff_gs2 - (m_gs2->m_sensorData.m_power - m_gsRef->m_sensorData.m_power);
	
	m_powerDiffResidual = r_powerDiff_gs1 * r_powerDiff_gs1 + r_powerDiff_gs2 * r_powerDiff_gs2;
	m_timeDiffResidual = r_timeDiff_gs1 * r_timeDiff_gs1 + r_timeDiff_gs2 * r_timeDiff_gs2;


	//���е��˴�֤����Ϊ��Ч�⣬Ȩ�ؼ���+1
	m_gs1->m_wCount += 1;
	m_gs2->m_wCount += 1;
	return true;
}

bool GSPair::HasValidTDOASolution_MPSTSD(const Scene* scene)
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

	//�ж�׼��2- �ж�·������Ч�ԣ�׷����Դ��
	RtLbsType delay_refSource = 0.0;
	RtLbsType power_refSource = 0.0;
	if (!m_gsRef->CalTDOAParameters_MPSTSD(m_targetSolution, scene)) {
		return false;
	}

	RtLbsType delay_source1 = 0.0;
	RtLbsType power_source1 = 0.0;
	if (!m_gs1->CalTDOAParameters_MPSTSD(m_targetSolution, scene)) {
		return false;
	}

	RtLbsType delay_source2 = 0.0;
	RtLbsType power_source2 = 0.0;
	if (!m_gs2->CalTDOAParameters_MPSTSD(m_targetSolution, scene)) {
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
	RtLbsType r_normalized_phi = m_phiResidual / max_r_phi;												/** @brief	��һ���ĽǶȲв�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_phi = 1.0 / (r_normalized_phi + 1e-4);													/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-4);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = static_cast<RtLbsType>(m_clusterSize) / static_cast<RtLbsType>(max_clusterNum);		/** @brief	����Ȩ��	*/
	RtLbsType w_total = (w.m_phiWeight * w_phi + w.m_powerWeight * w_powerDiff)  * m_clusterSize;						/** @brief	��Ȩ��	*/
	m_gs1->m_weight = std::max(m_gs1->m_weight, w_total);
	m_gs2->m_weight = std::max(m_gs2->m_weight, w_total);
	m_weight = w_total;
}

void GSPair::CalNormalizedWeightAndUpdate_TDOA(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum)
{
	RtLbsType r_normalized_timeDiff = m_phiResidual / max_r_timeDiff;									/** @brief	��һ����ʱ���в�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-6);										/** @brief	ʱ���Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-6);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	����Ȩ��	*/
	RtLbsType w_total = (w.m_timeWeight * w_timeDiff + w.m_phiWeight * w_powerDiff) * m_clusterSize * m_clusterSize;		/** @brief	��Ȩ��	*/
	if (m_gs1->m_weight < w_total) {
		m_gs1->m_weight = w_total;
	}
	if (m_gs2->m_weight < w_total) {
		m_gs2->m_weight = w_total;
	}
	m_weight = w_total;
}

void GSPair::CalNormalizedWeightAndUpdate_AOA_TDOA(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum)
{
	RtLbsType r_normalized_phi = m_phiResidual / max_r_phi;												/** @brief	��һ���ĽǶȲв�	*/
	RtLbsType r_normalized_timeDiff = m_phiResidual / max_r_timeDiff;									/** @brief	��һ����ʱ���в�	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	��һ���Ĺ��ʲ�в�	*/
	RtLbsType w_phi = 1.0 / (r_normalized_phi + 1e-6);													/** @brief	�Ƕ�Ȩ��	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-6);										/** @brief	ʱ���Ȩ��	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-6);										/** @brief	���ʲ�Ȩ��	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	����Ȩ��	*/
	RtLbsType w_total = (w.m_phiWeight * w_phi + w.m_timeWeight * w_timeDiff + w.m_powerWeight * w_powerDiff) * m_clusterSize * m_clusterSize;		/** @brief	��Ȩ��	*/
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
