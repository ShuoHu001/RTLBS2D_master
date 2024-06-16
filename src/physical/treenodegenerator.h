#ifndef RTLBS_TREENODEGENERATOR
#define RTLBS_TREENODEGENERATOR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "tree/raytreenode.h"
#include "result/result.h"
#include "result/lbsresult.h"
#include "geometry/scene.h"
#include "equipment/sensor.h"
#include "tree/lbstreenode.h"
#include "tree/gpu/treenodegpu.h"
#include "parallel/threadpool.h"


//CPU �������ڵ����
void TreeNodeGenerator_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, Result& result);

//CPU ������ڵ����
void TreeNodeGenerator_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, Result& result);

//GPU ������ڵ����
void TreeNodeGenerator_GPUMultiThread(const std::vector<std::vector<TreeNodeGPU*>>& gpuTreeNodes, const Scene* scene, Result& result);

#endif
