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

void GenerateAllTreeNodeAndConvertToCPUConvertPathNode(const std::vector<RayTreeNode*>& roots, std::vector<CPUConverterPathNode>& outNodes, int& maxDepth);			//�������ṹ�е����нڵ㲢����ת��ΪGPU�е�CPUConvertPathNode
void GenerateAllTreeNode(RayTreeNode* root, std::vector<PathNode*>* outNodes);				//�����������ڵ�
void GenerateAllLeafTreeNode(RayTreeNode* root, std::vector<PathNode*>* outNodes);			//����������֦�ڵ�
void GenerateMultiPath(RayTreeNode* root, std::vector<RayPath*>& outPaths); //������ȫ·��
void GenerateMultipathofPoint(RayTreeNode* root, Point2D rx, const Scene* scene, RtLbsType splitRadius, std::vector<RayPath*>& outPaths); //����rx��Χ��·��

#endif
