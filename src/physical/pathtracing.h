#ifndef RTLBS_PATHTRACING
#define RTLBS_PATHTRACING

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/struct.h"
#include "tree/raytreenode.h"
#include "tree/pathnode.h"
#include "geometry/ray2d.h"
#include "geometry/scene.h"
#include "geometry/Intersection2D.h"
#include "geometry/bbox2d.h"
#include "physical/propagationproperty.h"
#include "limitinfo.h"

#include "reflection.h"
#include "transmission.h"
#include "diffraction.h"
#include "raysplitting.h"


//全局射线追踪
void PathTrace(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot);

//定位服务下的射线追踪算法
void PathTraceLBS(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot);


//根据path中的路径进行计算,校正透射路径
bool PathTraceLite(RayPath& inpath);


#endif
