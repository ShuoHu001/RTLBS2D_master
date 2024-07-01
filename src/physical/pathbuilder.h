#ifndef RTLBS_PATHBUILDER
#define RTLBS_PATHBUILDER

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/point3d.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "geometry/scene.h"
#include "tree/raytreenode.h"
#include "tree/pathnode.h"
#include "tree/pathnode3d.h"
#include "tree/raypath.h"
#include "tree/raypath3d.h"
#include "tree/terraindiffractionpath.h"
#include "managers/logmanager.h"
#include "parallel/threadpool.h"
#include "tree/gpu/pathnodegpu.h"
#include "tree/gpu/raypathgpu.h"
#include "physical/treenodegenerator.h"
#include "parallel/threadpool.h"
#include "localization/gspaircluster.h"


class Result;
class RaytracingResult;

//CPU����·���ռ�
void PathBuilder_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, Result& result);

//CPU���·���ռ�
void PathBuilder_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, int16_t threadNum, Result& result);

//GPU���·���ռ�
void PathBuilder_GPUMultiThread(const std::vector<std::vector<PathNodeGPU*>>& gpuPathNodes, const Scene* scene, Result& result);

//����-����ྶ��Ϣ
void PathBuilder_DEBUG(const std::vector<RayTreeNode*>& vroots, const Scene* scene);

//ʵʱ��������׷��·����Ϣ��CPU���̣߳�-LBS�����õ�
void DirectlySetResultPath_CPUSingleThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, GSPairCluster* cluster);

//ʵʱ��������׷��·����Ϣ ��CPU���̣߳�-LBS�����õ�
void DirectlySetResultPath_CPUMultiThread(const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, uint16_t threadNum, std::vector<GSPairCluster>& clusters);





#endif
