#ifndef RTLBS_RAYTRACING
#define RTLBS_RAYTRACING

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/scene.h"
#include "geometry/ray2d.h"
#include "tree/pathnode.h"
#include "tree/raytreenode.h"
#include "tree/gpu/pathnodegpu.h"
#include "physical/limitinfo.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "physical/pathtracing.h"
#include "parallel/threadpool.h"
#include "geometry/gpu/segment2dgpu.h"
#include "geometry/gpu/wedge2dgpu.h"
#include "gpu/pathtracinggpu.h"
#include "utility/gpuutil.h"



void RayTracing_CPUSingleThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<RayTreeNode*>& vroots);		//���߳�����׷��

//��λģʽ�µĵ���������׷��
void RayTracingLBS_CPUSingleThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<RayTreeNode*>& vroots);		//���߳�����׷�٣�LBS

void RayTracing_CPUMultiThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, uint16_t threadNum, std::vector<RayTreeNode*>& vroots);

//��λģʽ�µĶ��������׷��
void RayTracingLBS_CPUMultiThread(SYSTEM_MODE systemMode, const std::vector<std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, uint16_t threadNum, std::vector<RayTreeNode*>& vroots);

//����֮ǰ��Ҫ��֤scene�е�gpu�ڴ�ת��
void RayTracing_GPUMultiThread(const std::vector < std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<std::vector<PathNodeGPU*>>& gpuPathNodes);


//GPU����׷��-ֻ�������ṹ
void RayTracing_GPUMultiThreadWithNode(const std::vector < std::vector<Ray2D>>& rays, const LimitInfo& limitInfo, bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, std::vector<std::vector<TreeNodeGPU*>>& gpuTreeNodes);

#endif
