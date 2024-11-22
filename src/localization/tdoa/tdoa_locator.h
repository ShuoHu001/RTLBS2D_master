#ifndef RTLBS_TDOA_LOCATOR
#define RTLBS_TDOA_LOCATOR

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point2d.h"
#include "localization/gspair.h"
#include "localization/gspaircluster.h"
#include "localization/lbsinfo.h"
#include "localization/lbsinfocluster.h"
#include "radiowave/raypath/raytreenode.h"
#include "scene/scene.h"
#include "equipment/sensor/sensordataprocessing.h"
#include "core/render/pathtracing/pathbuilder.h"
#include "core/render/pathtracing/gpu/pathbuildergpu.h"
#include "localization/raypathsetter.h"
#include "localization/gpu/raypathsettergpu.h"
#include "general/elevationmatrix.h"


//TDOA定位-多站单源单数据定位-保留
Point2D LBS_TDOA_LOCATOR_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData);

//TDOA定位-单站单源多数据定位-保留
Point2D LBS_TDOA_LOCATOR_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData);




#endif