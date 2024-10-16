#ifndef RTLBS_TREENODEGENERATOR
#define RTLBS_TREENODEGENERATOR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "radiowave/raypath/raytreenode.h"
#include "scene/scene.h"
#include "equipment/sensor/sensor.h"
#include "radiowave/raypath/gpu/treenodegpu.h"
#include "parallel/threadpool.h"
#include "radiowave/raypath/raypath.h"
#include "core/raypathrectifier/raypathrectifier.h"

class CPUConverterPathNode;

void GenerateAllTreeNodeAndConvertToCPUConvertPathNode(const std::vector<RayTreeNode*>& roots, std::vector<CPUConverterPathNode>& outNodes, int& maxDepth);			//产生树结构中的所有节点并进行转换为GPU中的CPUConvertPathNode
void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>* outNodes);				//产生所有树节点
void GenerateAllLeafTreeNode(RayTreeNode* root, std::vector<PathNode*>* outNodes);			//产生所有树枝节点
void GenerateMultiPath(RayTreeNode* root, std::vector<RayPath*>& outPaths); //产生完全路径
void GenerateMultipathofPoint(RayTreeNode* root, Point2D rx, const Scene* scene, RtLbsType splitRadius, std::vector<RayPath*>& outPaths); //产生rx周围的路径

#endif
