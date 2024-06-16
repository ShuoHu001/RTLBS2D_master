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


//ȫ������׷��
void PathTrace(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot);

//��λ�����µ�����׷���㷨
void PathTraceLBS(bool raySplitFlag, RtLbsType raySplitRadius, const Scene* scene, RayTreeNode*& vroot);


//����path�е�·�����м���,У��͸��·��
bool PathTraceLite(RayPath& inpath);


#endif
