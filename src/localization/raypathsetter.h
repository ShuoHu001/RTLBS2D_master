#ifndef RTLBS_RAYPATHSETTER
#define RTLBS_RAYPATHSETTER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"

#include "radiowave/raypath/raytreenode.h"
#include "radiowave/raypath/pathnode.h"
#include "radiowave/raypath/pathnode3d.h"
#include "radiowave/raypath/raypath.h"
#include "radiowave/raypath/raypath3d.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "core/render/pathtracing/treenodegenerator.h"
#include "parallel/threadpool.h"
#include "localization/gspaircluster.h"

//实时计算射线追踪路径信息（CPU单线程）-LBS方法用到
void DirectlySetResultPath_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, GSPairCluster* cluster);

//实时计算射线追踪路径信息 （CPU多线程）-LBS方法用到
void DirectlySetResultPath_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, uint16_t threadNum, std::vector<GSPairCluster>& clusters);


#endif