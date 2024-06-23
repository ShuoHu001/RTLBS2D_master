#ifndef RTLBS_TREENODEGENERATOR
#define RTLBS_TREENODEGENERATOR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "tree/raytreenode.h"
#include "result/lbsresult.h"
#include "geometry/scene.h"
#include "equipment/sensor.h"
#include "tree/lbstreenode.h"
#include "tree/gpu/treenodegpu.h"
#include "parallel/threadpool.h"
#include "tree/raypath.h"

class CPUConverterPathNode;
class Result;

//CPU 单核树节点遍历-AOA型
void TreeNodeGenerator_AOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, Result& result);

//CPU 单核树节点遍历-TDOA型
void TreeNodeGenerator_TDOA_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, Result& result);

//CPU 多核树节点遍历-AOA型
void TreeNodeGenerator_AOA_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, Result& result);

//CPU 多核树节点遍历-TDOA型
void TreeNodeGenerator_TDOA_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, int16_t threadNum, Result& result);



//GPU 多核树节点遍历
void TreeNodeGenerator_GPUMultiThread(const std::vector<std::vector<TreeNodeGPU*>>& gpuTreeNodes, const Scene* scene, Result& result);

void GenerateAllTreeNodeAndConvertToCPUConvertPathNode(const std::vector<RayTreeNode*>& roots, std::vector<CPUConverterPathNode>& outNodes, int& maxDepth);			//产生树结构中的所有节点并进行转换为GPU中的CPUConvertPathNode

#endif
