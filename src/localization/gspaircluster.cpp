#include "gspaircluster.h"

GSPairCluster::GSPairCluster()
    : m_isValid(true)
{
}

GSPairCluster::GSPairCluster(const GSPairCluster& cluster)
    : m_isValid(cluster.m_isValid)
    , m_pairs(cluster.m_pairs)
    , m_point(cluster.m_point)
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
    return *this;
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
    for (auto pair : m_pairs) {
        pair->m_phiResidual = r_phi;
        pair->m_powerDiffResidual = r_powerDiff;
        pair->m_nullDataNum = nullDataNum;
        pair->m_clusterSize = static_cast<int>(m_pairs.size());
    }
}

void GSPairCluster::SetElementTDOAResidual(RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum)
{
    for (auto pair : m_pairs) {
        pair->m_timeDiffResidual = r_timeDiff;
        pair->m_powerDiffResidual = r_powerDiff;
        pair->m_nullDataNum = nullDataNum;
        pair->m_clusterSize = static_cast<int>(m_pairs.size());
    }
}

void GSPairCluster::SetElementAOATDOAResidual(RtLbsType r_phi, RtLbsType r_timeDiff, RtLbsType r_powerDiff, int nullDataNum)
{
	for (auto pair : m_pairs) {
        pair->m_phiResidual = r_phi;
		pair->m_timeDiffResidual = r_timeDiff;
		pair->m_powerDiffResidual = r_powerDiff;
        pair->m_nullDataNum = nullDataNum;
        pair->m_clusterSize = static_cast<int>(m_pairs.size());
	}
}
