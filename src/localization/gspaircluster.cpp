#include "gspaircluster.h"


GSPairCluster::GSPairCluster()
    : m_isValid(true)
    , m_isDeviateSolution(false)
    , m_nearExtendNum(0)
    , m_farExtendNum(0)
    , m_deviateDistance(0.5)
    , m_residualFactor(1.0)
    , m_residual(0.0)
    , m_angularSpreadResidual(0.0)
    , m_delaySpreadRedisual(0.0)
    , m_phiResidual(0.0)
    , m_timeResidual(0.0)
    , m_timeDiffResidual(0.0)
    , m_powerResidual(0.0)
    , m_powerDiffResidual(0.0)
    , m_nullDataNum(0)
    , m_weight(0.0)
{
}

GSPairCluster::GSPairCluster(const GSPairCluster& cluster)
    : m_isValid(cluster.m_isValid)
    , m_isDeviateSolution(cluster.m_isDeviateSolution)
    , m_deviateDistance(cluster.m_deviateDistance)
    , m_nearExtendNum(cluster.m_nearExtendNum)
    , m_farExtendNum(cluster.m_farExtendNum)
    , m_pairs(cluster.m_pairs)
    , m_point(cluster.m_point)
    , m_aroundPoints(cluster.m_aroundPoints)
    , m_residualFactor(cluster.m_residualFactor)
    , m_residual(cluster.m_residual)
    , m_rtResult(cluster.m_rtResult)
    , m_angularSpreadResidual(cluster.m_angularSpreadResidual)
    , m_delaySpreadRedisual(cluster.m_delaySpreadRedisual)
    , m_phiResidual(cluster.m_phiResidual)
    , m_timeResidual(cluster.m_timeResidual)
    , m_timeDiffResidual(cluster.m_timeDiffResidual)
    , m_powerResidual(cluster.m_powerResidual)
    , m_powerDiffResidual(cluster.m_powerDiffResidual)
    , m_nullDataNum(cluster.m_nullDataNum)
    , m_weight(cluster.m_weight)
{
}

GSPairCluster::~GSPairCluster()
{
    m_aroundPoints.clear();
    std::vector<Point2D>().swap(m_aroundPoints);

    for (auto& curResults : m_rtResult) {
        curResults.clear();
        std::vector<RaytracingResult>().swap(curResults);
    }
    std::vector<std::vector<RaytracingResult>>().swap(m_rtResult);
}

GSPairCluster GSPairCluster::operator=(const GSPairCluster& cluster)
{
    m_isValid = cluster.m_isValid;
    m_isDeviateSolution = cluster.m_isDeviateSolution;
    m_nearExtendNum = cluster.m_nearExtendNum;
    m_farExtendNum = cluster.m_farExtendNum;
    m_deviateDistance = cluster.m_deviateDistance;
    m_pairs = cluster.m_pairs;
    m_point = cluster.m_point;
    m_aroundPoints = cluster.m_aroundPoints;
    m_residualFactor = cluster.m_residualFactor;
    m_residual = cluster.m_residual;
    m_rtResult = cluster.m_rtResult;
    m_angularSpreadResidual = cluster.m_angularSpreadResidual;
    m_delaySpreadRedisual = cluster.m_delaySpreadRedisual;
    m_phiResidual = cluster.m_phiResidual;
    m_timeResidual = cluster.m_timeResidual;
    m_timeDiffResidual = cluster.m_timeDiffResidual;
    m_powerResidual = cluster.m_powerResidual;
    m_powerDiffResidual = cluster.m_powerDiffResidual;
    m_nullDataNum = cluster.m_nullDataNum;
    m_weight = cluster.m_weight;
    return *this;
}

void GSPairCluster::UpdateGSPairBelongingCluster()
{
    for (auto& curPair : m_pairs) {
        curPair->m_belongingPairCluster = this;
    }
}

RtLbsType GSPairCluster::CalTDOAResidualFactor()
{
    if (m_pairs.size() == 1) {
        m_residualFactor = 1.0;
    }
    else {
        RtLbsType mean_r_powerDiff = 0.0;                           /** @brief	���ʲ��ֵ	*/
        for (auto& curPair : m_pairs) {
            mean_r_powerDiff += curPair->m_powerDiffResidual;
        }
        mean_r_powerDiff /= static_cast<int>(m_pairs.size());
        RtLbsType std_r_powerDiff = 0.0;
		for (auto& curPair : m_pairs) {
            std_r_powerDiff += (curPair->m_powerDiffResidual - mean_r_powerDiff) * (curPair->m_powerDiffResidual - mean_r_powerDiff);
		}
        std_r_powerDiff /= static_cast<int>(m_pairs.size());
        std_r_powerDiff = sqrt(std_r_powerDiff);
        m_residualFactor = std_r_powerDiff / static_cast<int>(m_pairs.size());      //����в�����
    }
    return m_residualFactor;
}

void GSPairCluster::ExtendAroundPoint(bool expandFlag, const ElevationMatrix& lbsShiftErrorMatrix, const Scene* scene)
{
    if (expandFlag == true) {
		
        RtLbsType offset = lbsShiftErrorMatrix.GetValue(m_point);                   //����λ�ƾ�����չ����
        m_deviateDistance = offset;

		Point2D NorthPoint = m_point + Point2D(0, offset);                          /** @brief	����	*/
		Point2D SouthPoint = m_point + Point2D(0, -offset);                         /** @brief	�ϵ�	*/
		Point2D WestPoint = m_point + Point2D(-offset, 0);                          /** @brief	����	*/
		Point2D EastPoint = m_point + Point2D(offset, 0);                           /** @brief	����	*/

		RtLbsType offset1 = 0.5;                                                    /** @brief	��б��ƫ����,�ṩ΢��ƫ��	*/
		Point2D NorthWestPoint = m_point + Point2D(-offset1, offset1);              /** @brief	������	*/
		Point2D NorthEastPoint = m_point + Point2D(offset1, offset1);               /** @brief	������	*/
		Point2D SouthWestPoint = m_point + Point2D(-offset1, -offset1);             /** @brief	���ϵ�	*/
		Point2D SouthEastPoint = m_point + Point2D(offset1, -offset1);              /** @brief	���ϵ�	*/

        m_aroundPoints.push_back(m_point);

        if (offset == 0.0) {
			if (scene->IsValidPoint(NorthWestPoint)) {
				m_aroundPoints.push_back(NorthWestPoint);
                m_nearExtendNum++;
			}
			if (scene->IsValidPoint(NorthEastPoint)) {
				m_aroundPoints.push_back(NorthEastPoint);
                m_nearExtendNum++;
			}
			if (scene->IsValidPoint(SouthWestPoint)) {
				m_aroundPoints.push_back(SouthWestPoint);
                m_nearExtendNum++;
			}
			if (scene->IsValidPoint(SouthEastPoint)) {
				m_aroundPoints.push_back(SouthEastPoint);
                m_nearExtendNum++;
			}
			m_rtResult.resize(m_aroundPoints.size(), std::vector<RaytracingResult>());
            return;
        }

		if (scene->IsValidPoint(NorthWestPoint)) {
			m_aroundPoints.push_back(NorthWestPoint);
            m_nearExtendNum++;
		}
		if (scene->IsValidPoint(NorthEastPoint)) {
			m_aroundPoints.push_back(NorthEastPoint);
            m_nearExtendNum++;
		}
		if (scene->IsValidPoint(SouthWestPoint)) {
			m_aroundPoints.push_back(SouthWestPoint);
            m_nearExtendNum++;
		}
		if (scene->IsValidPoint(SouthEastPoint)) {
			m_aroundPoints.push_back(SouthEastPoint);
            m_nearExtendNum++;
		}
		if (scene->IsValidPoint(NorthPoint)) {
			m_aroundPoints.push_back(NorthPoint);
			m_farExtendNum++;
		}
		if (scene->IsValidPoint(SouthPoint)) {
			m_aroundPoints.push_back(SouthPoint);
			m_farExtendNum++;
		}
		if (scene->IsValidPoint(WestPoint)) {
			m_aroundPoints.push_back(WestPoint);
			m_farExtendNum++;
		}
		if (scene->IsValidPoint(EastPoint)) {
			m_aroundPoints.push_back(EastPoint);
			m_farExtendNum++;
		}
        m_rtResult.resize(m_aroundPoints.size(), std::vector<RaytracingResult>());
    }
    else {
        m_aroundPoints.resize(1);
        m_rtResult.resize(1, std::vector<RaytracingResult>());
        m_aroundPoints[0] = m_point;
    }
   
}

void GSPairCluster::CalTDOAResidual(RtLbsType maxResidualFactor)
{
    if (m_residualFactor != 1.0) {
        m_residualFactor /= maxResidualFactor;
    }

    RtLbsType mean_r_powerDiff = 0.0;                        /** @brief	����ʲ�в�	*/
    for (auto& curPair : m_pairs) {
        mean_r_powerDiff += curPair->m_powerDiffResidual;
    }
    mean_r_powerDiff /= static_cast<int>(m_pairs.size());
    
    m_residual = mean_r_powerDiff * m_residualFactor;        //���²в�

}

bool GSPairCluster::CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold)
{
    //����pair��cluster�е�ÿ�����ݵľ��룬�����ڴ��ھ������޵���ֵ���򷵻�false
    bool addFlag = true;
    if (Distance(pair->m_targetSolution, m_point) > threshold) {
        addFlag = false;
    }
    return addFlag;
}

void GSPairCluster::CalClusterPosition()
{
    m_point = { 0,0 };
    for (auto pair : m_pairs) {
        m_point += pair->m_targetSolution;
    }
    m_point /= static_cast<RtLbsType>(m_pairs.size());
}

void GSPairCluster::SetElementClusterId(int Id)
{
    for (auto pair : m_pairs) {
        pair->m_clusterId = Id;
    }
}

void GSPairCluster::SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, RtLbsType r_angularSpreadResidual, int nullDataNum)
{
    m_phiResidual = r_phi;
    m_powerDiffResidual = r_powerDiff;
    m_nullDataNum = nullDataNum;
    for (auto pair : m_pairs) {
        pair->m_phiResidual = r_phi;
        pair->m_powerDiffResidual = r_powerDiff;
        pair->m_angularSpreadResidual = r_angularSpreadResidual;
        pair->m_nullDataNum = nullDataNum;
        pair->m_clusterSize = static_cast<int>(m_pairs.size());
    }
}

void GSPairCluster::SetElementTOAResidual(RtLbsType r_time, RtLbsType r_power, int nullDataNum)
{
	m_timeResidual = r_time;
	m_powerResidual = r_power;
	m_nullDataNum = nullDataNum;
	for (auto pair : m_pairs) {
		pair->m_timeResidual = r_time;
		pair->m_powerResidual = r_power;
		pair->m_nullDataNum = nullDataNum;
		pair->m_clusterSize = static_cast<int>(m_pairs.size());
	}
}

void GSPairCluster::SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum)
{
    m_timeDiffResidual = r_timeDiff;
    m_powerDiffResidual = r_powerDiff;
    m_nullDataNum = nullDataNum;
    for (auto pair : m_pairs) {
        pair->m_timeDiffResidual = r_timeDiff;
        pair->m_powerDiffResidual = r_powerDiff;
        pair->m_nullDataNum = nullDataNum;
        pair->m_clusterSize = static_cast<int>(m_pairs.size());
    }
}

void GSPairCluster::SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum)
{
    m_phiResidual = r_phi;
    m_timeDiffResidual = r_timeDiff;
    m_powerDiffResidual = r_powerDiff;
    m_nullDataNum = nullDataNum;
	for (auto pair : m_pairs) {
        pair->m_phiResidual = r_phi;
		pair->m_timeDiffResidual = r_timeDiff;
		pair->m_powerDiffResidual = r_powerDiff;
        pair->m_nullDataNum = nullDataNum;
        pair->m_clusterSize = static_cast<int>(m_pairs.size());
	}
}

void GSPairCluster::SetElementAOATOAResidual(RtLbsType r_phi, RtLbsType r_time, RtLbsType r_power, int nullDataNum)
{
	m_phiResidual = r_phi;
	m_timeResidual = r_time;
	m_powerResidual = r_power;
	m_nullDataNum = nullDataNum;
	for (auto pair : m_pairs) {
		pair->m_phiResidual = r_phi;
		pair->m_timeResidual = r_time;
		pair->m_powerResidual = r_power;
		pair->m_nullDataNum = nullDataNum;
		pair->m_clusterSize = static_cast<int>(m_pairs.size());
	}
}

void GSPairCluster::SetInValidState()
{
    m_isValid = false;
    for (auto& pair : m_pairs) {
        pair->m_isValid = false;
    }
}

void GSPairCluster::CalNormalizedAOAWeight(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w)
{
    RtLbsType weight_phi = max_r_phi / (m_phiResidual + 1e-4);
    RtLbsType weight_powerDiff = max_r_powerDiff / (m_powerDiffResidual + 1e-4);
    m_weight = (weight_phi * w.m_phiWeight + weight_powerDiff * w.m_powerWeight) * m_pairs.size();
}

void GSPairCluster::CalNormalizedTDOAWeight(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w)
{
	RtLbsType weight_timeDiff = max_r_timeDiff / (m_timeDiffResidual + 1e-4);
	RtLbsType weight_powerDiff = max_r_powerDiff / (m_powerDiffResidual + 1e-4);
    m_weight = (weight_timeDiff * w.m_timeWeight + weight_powerDiff * w.m_powerWeight) * m_pairs.size();
}

void GSPairCluster::CalNormalizedAOATDOAWeight(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w)
{
    RtLbsType weight_phi = max_r_phi / (m_phiResidual + 1e-4);
	RtLbsType weight_timeDiff = max_r_timeDiff / (m_timeDiffResidual + 1e-4);
	RtLbsType weight_powerDiff = max_r_powerDiff / (m_powerDiffResidual + 1e-4);
    m_weight = (weight_phi * w.m_phiWeight + weight_timeDiff * w.m_timeWeight + weight_powerDiff * w.m_powerWeight) * m_pairs.size();
}

void GSPairCluster::GetNonRepeatGeneralSource(std::vector<GeneralSource*>& sources)
{
    std::unordered_map<size_t, GeneralSource*> sourceMap;

    for (auto& curPair : m_pairs) {
        GeneralSource* gs1 = curPair->m_gs1;
        GeneralSource* gs2 = curPair->m_gs2;
        size_t hash1 = gs1->GetHash();
        size_t hash2 = gs2->GetHash();
        auto pos1 = sourceMap.find(hash1);
        auto pos2 = sourceMap.find(hash2);
        bool gs1_repeatFlag = false;                //����Դ1�ظ�״̬
        bool gs2_repeatFlag = false;                //����Դ2�ظ�״̬
        if (pos1 == sourceMap.end()) {              //hash1δ�ҵ��ظ���
            sourceMap[hash1] = gs1;
        }
        else {
            GeneralSource*& existSource = pos1->second;
            if (gs1 != existSource) {               //������Դ�ظ����򲻽��м���
				existSource->m_sensorData.m_phi += gs1->m_sensorData.m_phi;
				existSource->m_phiRepeatCount += 1;
				gs1_repeatFlag = true;
            }
        }

        if (pos2 == sourceMap.end()) {              //hash2δ�ҵ��ظ���
            sourceMap[hash2] = gs2;
        }
        else {
			GeneralSource*& existSource = pos2->second;
            if (gs2 != existSource) {               //������Դ�ظ����򲻽��м���
				existSource->m_sensorData.m_phi += gs2->m_sensorData.m_phi;
				existSource->m_phiRepeatCount += 1;
				gs2_repeatFlag = true;
            }
        }

        if (gs1_repeatFlag && gs2_repeatFlag) {     //����������Դ�������ظ�״̬����ǰpair���ظ��ģ��趨��ǰpair��Ч��Ϊfalse
            curPair->m_isValid = false;
        }
    }

	m_pairs.erase(std::remove_if(m_pairs.begin(), m_pairs.end(), [](const GSPair* pair) {           //���cluster����Ч�Ĺ���Դ��
		return pair->m_isValid == false;
		}), m_pairs.end());

    for (auto& data : sourceMap) {                                      //������ظ�����Դ
        GeneralSource* existSource = data.second;
        existSource->UpdateEvenPhiValue();                              //����phiֵ
        existSource->m_phiRepeatCount = 1;                              //����phi�ظ�����
        sources.push_back(existSource);
    }



}

void GSPairCluster::CalculateRefSourceCount() const
{
    for (auto& curPair : m_pairs) {
        curPair->m_gsRef->m_wCount += 1;
    }
}
