#include "gspaircluster.h"

GSPairCluster::GSPairCluster()
    : m_isValid(true)
    , m_residualFactor(1.0)
    , m_residual(0.0)
    , m_phiResidual(0.0)
    , m_timeResidual(0.0)
    , m_timeDiffResidual(0.0)
    , m_powerDiffResidual(0.0)
    , m_nullDataNum(0)
    , m_weight(0.0)
{
}

GSPairCluster::GSPairCluster(const GSPairCluster& cluster)
    : m_isValid(cluster.m_isValid)
    , m_pairs(cluster.m_pairs)
    , m_point(cluster.m_point)
    , m_aroundPoints(cluster.m_aroundPoints)
    , m_residualFactor(cluster.m_residualFactor)
    , m_residual(cluster.m_residual)
    , m_rtResult(cluster.m_rtResult)
    , m_phiResidual(cluster.m_phiResidual)
    , m_timeResidual(cluster.m_timeResidual)
    , m_timeDiffResidual(cluster.m_timeDiffResidual)
    , m_powerDiffResidual(cluster.m_powerDiffResidual)
    , m_nullDataNum(cluster.m_nullDataNum)
    , m_weight(cluster.m_weight)
{
}

GSPairCluster::~GSPairCluster()
{
}

GSPairCluster GSPairCluster::operator=(const GSPairCluster& cluster)
{
    m_isValid = cluster.m_isValid;
    m_pairs = cluster.m_pairs;
    m_point = cluster.m_point;
    m_aroundPoints = cluster.m_aroundPoints;
    m_residualFactor = cluster.m_residualFactor;
    m_residual = cluster.m_residual;
    m_rtResult = cluster.m_rtResult;
    m_phiResidual = cluster.m_phiResidual;
    m_timeResidual = cluster.m_timeResidual;
    m_timeDiffResidual = cluster.m_timeDiffResidual;
    m_powerDiffResidual = cluster.m_powerDiffResidual;
    m_nullDataNum = cluster.m_nullDataNum;
    m_weight = cluster.m_weight;
    return *this;
}

RtLbsType GSPairCluster::CalTDOAResidualFactor()
{
    if (m_pairs.size() == 1) {
        m_residualFactor = 1.0;
    }
    else {
        RtLbsType mean_r_powerDiff = 0.0;                           /** @brief	功率差均值	*/
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
        m_residualFactor = std_r_powerDiff / static_cast<int>(m_pairs.size());      //计算残差因子
    }
    return m_residualFactor;
}

void GSPairCluster::ExtendNearPoint(bool expandFlag, const Scene* scene)
{
    if (expandFlag == true) {
		m_aroundPoints.reserve(5);
		RtLbsType offset = 0.5;                                 /** @brief	位移0.5m	*/
        m_aroundPoints.push_back(m_point);
        Point2D upPoint = m_point + Point2D(0, offset);
        Point2D downPoint = m_point + Point2D(0, -offset);
        Point2D leftPoint = m_point + Point2D(-offset, 0);
        Point2D rightPoint = m_point + Point2D(offset, 0);
        if (scene->IsValidPoint(upPoint)) {
            m_aroundPoints.push_back(upPoint);
        }
        if (scene->IsValidPoint(downPoint)) {
            m_aroundPoints.push_back(downPoint);
        }
        if (scene->IsValidPoint(leftPoint)) {
            m_aroundPoints.push_back(leftPoint);
        }
        if (scene->IsValidPoint(rightPoint)) {
            m_aroundPoints.push_back(rightPoint);
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

    RtLbsType mean_r_powerDiff = 0.0;                        /** @brief	最大功率差残差	*/
    for (auto& curPair : m_pairs) {
        mean_r_powerDiff += curPair->m_powerDiffResidual;
    }
    mean_r_powerDiff /= static_cast<int>(m_pairs.size());
    
    m_residual = mean_r_powerDiff * m_residualFactor;        //更新残差

}

bool GSPairCluster::CanAddToClusterByDistance(GSPair* pair, RtLbsType threshold)
{
    //计算pair与cluster中的每个数据的距离，若存在大于距离门限的数值，则返回false
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

void GSPairCluster::SetElementAOAResidual(RtLbsType r_phi, RtLbsType r_powerDiff, int nullDataNum)
{
    m_phiResidual = r_phi;
    m_powerDiffResidual = r_powerDiff;
    m_nullDataNum = nullDataNum;
    for (auto pair : m_pairs) {
        pair->m_phiResidual = r_phi;
        pair->m_powerDiffResidual = r_powerDiff;
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
