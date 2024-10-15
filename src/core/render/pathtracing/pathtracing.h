#ifndef RTLBS_PATHTRACING
#define RTLBS_PATHTRACING

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/struct.h"
#include "radiowave/raypath/raytreenode.h"
#include "radiowave/raypath/pathnode.h"
#include "geometry/ray2d.h"
#include "scene/scene.h"
#include "geometry/Intersection2D.h"
#include "geometry/bbox2d.h"
#include "configuration/radiowave/propagation/propagationproperty.h"
#include "configuration/radiowave/propagation/limitinfo.h"

#include "radiowave/propagation/reflection.h"
#include "radiowave/propagation/transmission.h"
#include "radiowave/propagation/diffraction.h"
#include "raysplitting.h"


//全局射线追踪
void PathTrace(bool raySplitFlag, RtLbsType raySplitRadius, int diffractRayNum, const Scene* scene, RayTreeNode*& vroot);

//定位服务下的射线追踪算法
void PathTraceLBS(bool raySplitFlag, RtLbsType raySplitRadius, int diffractRayNum, const Scene* scene, RayTreeNode*& vroot);


//根据path中的路径进行计算,校正透射路径
bool PathTraceLite(RayPath*& inpath);


#endif
