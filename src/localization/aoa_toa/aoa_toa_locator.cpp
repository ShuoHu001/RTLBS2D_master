#include "aoa_toa_locator.h"

Point2D LBS_AOA_TOA_Locator_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor)
{
    std::vector<LBSInfo*>& lbsInfos = lbsInfoCluster.m_infos;
    //0-计算广义源的位置

}
