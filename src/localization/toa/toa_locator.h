#ifndef RTLBS_TOALOCATOR
#define RTLBS_TOALOCATOR

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point2d.h"
#include "localization/gspair.h"
#include "localization/gspaircluster.h"
#include "localization/lbsinfo.h"
#include "radiowave/raypath/raytreenode.h"
#include "scene/scene.h"
#include "equipment/sensor/sensordataprocessing.h"
#include "core/render/pathtracing/pathbuilder.h"
#include "core/render/pathtracing/gpu/pathbuildergpu.h"
#include "localization/lbsinfocluster.h"
#include "localization/raypathsetter.h"
#include "localization/gpu/raypathsettergpu.h"
#include "toa_solver.h"
#include "general/elevationmatrix.h"

//TOA定位-多站点单源单数据定位
Point2D LBS_TOA_Locator_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor);

//TOA定位-单站点单源单数据定位--暂缓
Point2D LBS_TOA_Locator_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, HARDWAREMODE hardwareMode, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData, uint16_t threadNum, RtLbsType gsPairClusterThreshold, bool extendAroundPointState, const WeightFactor& weightFactor);
#endif