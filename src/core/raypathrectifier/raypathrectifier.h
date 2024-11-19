#ifndef RTLBS_RAYPATHRECTIFIER
#define RTLBS_RAYPATHRECTIFIER

#include "rtlbs.h"
#include "utility/define.h"
#include "radiowave/raypath/raypath.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "scene/scene.h"
#include "core/render/pathtracing/pathtracing.h"
#include "core/render/pathtracing/gpu/pathtracinggpu.h"

bool RectifyRayPath(const Scene* scene, RayPath*&path, const Point2D& p);															//修正RayPath
bool RectifyGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p);														//修正RayPathGPU


bool _isValidAndRectifyCommonRayPath(const Scene* scene, RayPath*& path, const Point2D& p);											//检测并修正常规路径（反射路径和透射路径）
bool _isValidAndRectifyRefractedRayPath(const Scene* scene, RayPath*& path, const Point2D& p);										//检测并修正透射路径
bool _isValidAndRectifyCommonGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p);									//检测并修正GPU常规路径（反射路径和透射路径）
bool _isValidAndRectifyRefractedGPURayPath(const Scene* scene, RayPathGPU& path, const Point2D& p);									//检测并修正GPU透射路径


#endif
