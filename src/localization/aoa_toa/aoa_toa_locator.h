#ifndef RTLBS_AOATOALOCATOR
#define RTLBS_AOATOALOCATOR

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
#include "localization/aoa/aoasolver.h"
#include "aoa_toa_solver.h"
#include "general/elevationmatrix.h"


//AOATOA定位-多站单源单数据定位
Point2D LBS_AOA_TOA_Locator_MPSTSD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData);

//AOATOA定位-单站单源多数据定位
Point2D LBS_AOA_TOA_Locator_SPSTMD(LBSInfoCluster& lbsInfoCluster, const std::vector<RayTreeNode*>& vroots, const Scene* scene, const LocalizeConfig& lbsConfig, const ElevationMatrix& lbsShiftErrorMatrix, RtLbsType splitRadius, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData);
#endif
