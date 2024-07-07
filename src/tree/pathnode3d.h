#ifndef RTLBS_PATHNODE3D
#define RTLBS_PATHNODE3D

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/segment2d.h"
#include "geometry/wedge2d.h"
#include "geometry/terrain/terrainfacet.h"
#include "geometry/point3d.h"
#include "pathnode.h"
#include "gpu/pathnodegpu.h"
#include "material/material.h"
#include "material/materiallibrary.h"



//版本0-0-0-0 暂不考虑广义源和传播距离
class PathNode3D {
public:
	unsigned m_depth;																												/** @brief	节点深度	*/
	PATHNODETYPE m_type;																											/** @brief	节点类型	*/
	Material* m_mat;																												/** @brief	节点所在材质	*/
	Point3D m_point;																												/** @brief	节点坐标	*/
	Point2D m_gs2D;																													/** @brief	末端广义源(二维)	*/
	const Segment2D* m_primitive;																									/** @brief	节点所在面元的指针	*/
	const Wedge2D* m_wedge;																											/** @brief	节点所在wedge的指针	*/
	const TerrainFacet* m_terrainFacet;																								/** @brief	节点所在的地形面元的指针	*/
public:
	PathNode3D();
	PathNode3D(const PathNode& node, RtLbsType h);																					//由二维PathNode节点转换为三维PathNode节点的构造函数
	PathNode3D(const Point3D& point, PATHNODETYPE type, int depth = 0);																/** @brief	起始节点或终止节点的构造函数	*/
	PathNode3D(const Point3D& point, PATHNODETYPE type, const Segment2D* segment, int depth = 0);									/** @brief	反射节点的构造函数	*/
	PathNode3D(const Point3D& point, PATHNODETYPE type, const TerrainFacet* facet, int depth = 0);									/** @brief	地面反射节点的构造函数	*/
	PathNode3D(const Point3D& point, PATHNODETYPE type, const Wedge2D* wedge, int depth = 0);										/** @brief	绕射节点的构造函数	*/
	~PathNode3D();																													//析构函数
	void ConvertBy(const PathNode& node, RtLbsType h);																				//由PathNode转换为PathNode3D
	void ConvertBy(const PathNodeGPU& node, RtLbsType h, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges);			//由PathNodeGPU转换为PathNode3D
	std::string ToString() const;																									//路径节点转换为字符串
};

#endif
