#ifndef RTLBS_RAYPATHRECTIFIER
#define RTLBS_RAYPATHRECTIFIER

#include "rtlbs.h"
#include "utility/define.h"
#include "radiowave/raypath/raypath.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "scene/scene.h"
#include "core/render/pathtracing/pathtracing.h"
#include "core/render/pathtracing/gpu/pathtracinggpu.h"

bool RectifyRayPath(const Scene* scene, RayPath*&path, const Point2D& p);															//����RayPath
bool RectifyGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p);														//����RayPathGPU


bool _isValidAndRectifyCommonRayPath(const Scene* scene, RayPath*& path, const Point2D& p);											//��Ⲣ��������·��������·����͸��·����
bool _isValidAndRectifyRefractedRayPath(const Scene* scene, RayPath*& path, const Point2D& p);										//��Ⲣ����͸��·��
bool _isValidAndRectifyCommonGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p);									//��Ⲣ����GPU����·��������·����͸��·����
bool _isValidAndRectifyRefractedGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p);									//��Ⲣ����GPU͸��·��


#endif
