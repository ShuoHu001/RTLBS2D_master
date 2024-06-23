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
	if (!_calAOASolution()) {											//若无有效AOA解，则返回false
		m_isValid = false;
		return false;
	}

	//判定准测0-解是否和传感器位置重复
	for (auto curSensor : scene->m_sensors) {
		Point2D curSensorPoint = curSensor->GetPosition2D();
		if (Distance(curSensorPoint, m_targetSolution) < 1e-1) {			//若解距离与传感器位置小于10cm，则表明解无效
			return false;
		}
	}


	//判定准测1- 是否处于环境的无效位置(超出环境边界或在建筑内部)
	if (!scene->IsValidPoint(m_targetSolution)) {
		m_isValid = false;
		return false;
	}

	//判定准则2- 在满足AOA规则的条件下，广义源所在传播节点与目标间的构成的末端路径是否被环境所遮挡
	const Point2D& np1 = m_gs1->m_nodePosition;							/** @brief	广义源1所在节点坐标	*/
	const Point2D& np2 = m_gs2->m_nodePosition;							/** @brief	广义源2所在节点坐标	*/

	Segment2D segment1(m_targetSolution, np1);							/** @brief	组合测试线段1	*/
	Segment2D segment2(m_targetSolution, np2);							/** @brief	组合测试线段2	*/

	if (scene->GetIntersect(segment1, nullptr)) {						//若线段1与环境相交，则表明解无效
		m_isValid = false;
		return false;
	}
	if (scene->GetIntersect(segment2, nullptr)) {						//若线段2与环境相交，则表明解无效
		m_isValid = false;
		return false;
	}

	//运行到此处证明解为有效解，权重计数+1
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

	//判定准测0-解是否和传感器位置重复
	for (auto curSensor : scene->m_sensors) {
		Point2D curSensorPoint = curSensor->GetPosition2D();
		if (Distance(curSensorPoint, m_targetSolution) < 1e-1) {			//若解距离与传感器位置小于10cm，则表明解无效
			return false;
		}
	}


	//判定准测1- 是否处于环境的无效位置(超出环境边界或在建筑内部)
	if (!scene->IsValidPoint(m_targetSolution)) {
		m_isValid = false;
		return false;
	}

	//判定准则2- 在满足AOA规则的条件下，广义源所在传播节点与目标间的构成的末端路径是否被环境所遮挡
	const Point2D& np1 = m_gs1->m_position;								/** @brief	广义源1坐标	*/
	const Point2D& np2 = m_gs2->m_position;								/** @brief	广义源2坐标	*/

	Segment2D segment1(m_targetSolution, np1);							/** @brief	组合测试线段1	*/
	Segment2D segment2(m_targetSolution, np2);							/** @brief	组合测试线段2	*/

	if (m_gs1->m_type == NODE_REFL) {									//反射广义源处理
		Intersection2D intersect;
		if (!scene->GetIntersect(segment1, &intersect)) {				//线段与反射面不相交，代表解集所构成的路径不存在
			return false;
		}
		if (intersect.m_segment->m_id != m_gs1->m_segment->m_id) {		//若线段1与环境相交，其交点所在环境线段与产生广义源的线段不同，则代表解无效
			m_isValid = false;
			return false;
		}
	}
	else {																//绕射、根节点广义源处理
		Intersection2D intersect;
		if (scene->GetIntersect(segment1, &intersect)) {				//若目标坐标与绕射、根节点广义源间构成的线段与环境相交，则表明解无效
			m_isValid = false;
			return false;
		}
	}
	
	if (m_gs2->m_type == NODE_REFL) {									//反射广义源处理
		Intersection2D intersect;
		if (!scene->GetIntersect(segment2, &intersect)) {				//线段与反射面不相交，代表解集所构成的路径不存在
			return false;
		}
		if (intersect.m_segment->m_id != m_gs2->m_segment->m_id) {		//若线段2与环境相交，其交点所在环境线段与产生广义源的线段不同，则代表解无效
			m_isValid = false;
			return false;
		}
	}
	else {
		Intersection2D intersect;
		if (scene->GetIntersect(segment2, &intersect)) {				//若目标坐标与绕射、根节点广义源间构成的线段与环境相交，则表明解无效
			m_isValid = false;
			return false;
		}
	}
	
	//运行到此处证明解为有效解，权重计数+1
	m_gs1->m_wCount += 1;
	m_gs2->m_wCount += 1;

	return true;
}

void GSPair::CalNormalizedWeightAndUpdate_AOA(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, int max_clusterNum)
{
	RtLbsType r_normalized_phi = m_phiResidual / max_r_phi;												/** @brief	归一化的角度残差	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	归一化的功率差残差	*/
	RtLbsType w_phi = 1.0 / (r_normalized_phi + 1e-4);													/** @brief	角度权重	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-4);										/** @brief	功率差权重	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	聚类权重	*/
	RtLbsType w_total = ONE_THIRD * w_phi + ONE_THIRD * w_powerDiff + ONE_THIRD * w_cluster;			/** @brief	总权重	*/
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
	RtLbsType r_normalized_timeDiff = m_phiResidual / max_r_timeDiff;									/** @brief	归一化的时间差残差	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	归一化的功率差残差	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-6);										/** @brief	时间差权重	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-6);										/** @brief	功率差权重	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	聚类权重	*/
	RtLbsType w_total = ONE_THIRD * w_timeDiff + ONE_THIRD * w_powerDiff + +ONE_THIRD * w_cluster;		/** @brief	总权重	*/
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
	RtLbsType r_normalized_phi = m_phiResidual / max_r_phi;												/** @brief	归一化的角度残差	*/
	RtLbsType r_normalized_timeDiff = m_phiResidual / max_r_timeDiff;									/** @brief	归一化的时间差残差	*/
	RtLbsType r_normalized_powerDiff = m_powerDiffResidual / max_r_powerDiff;							/** @brief	归一化的功率差残差	*/
	RtLbsType w_phi = 1.0 / (r_normalized_phi + 1e-6);													/** @brief	角度权重	*/
	RtLbsType w_timeDiff = 1.0 / (r_normalized_timeDiff + 1e-6);										/** @brief	时间差权重	*/
	RtLbsType w_powerDiff = 1.0 / (r_normalized_powerDiff + 1e-6);										/** @brief	功率差权重	*/
	RtLbsType w_cluster = m_clusterSize / max_clusterNum;												/** @brief	聚类权重	*/
	RtLbsType w_total = 0.25 * w_phi + 0.25 * w_timeDiff + 0.25 * w_powerDiff + 0.25 * w_cluster;		/** @brief	总权重	*/
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

	if (std::abs(theta1 - theta2) < EPSILON) {			//若角度相等，不会有解
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
	//采用ceres求解TDOA解
	TDOASolver tdoaSolver;
	tdoaSolver.SetGeneralSource(m_gsRef, m_gs1, m_gs2);				//设定广义源
	RtLbsType accuracy = tdoaSolver.Solving_LS(m_targetSolution);
	if (accuracy > EPSILON) {													//若精度高于EPSILON则认定为该组方程为无解情况
		return false;
	}
	return true;
}
