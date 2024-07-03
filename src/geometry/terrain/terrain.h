#ifndef RTLBS_TERRAIN
#define RTLBS_TERRAIN

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/ray2d.h"
#include "geometry/Intersection2D.h"
#include "geometry/bbox2d.h"

#include "terrainfacet.h"
#include "terrainsegment.h"
#include "terrainprofile.h"

#include "physical/propagationproperty.h"
#include "configuration/terrainconfig.h"
#include "utility/fileoperation.h"
#include "resources/supports.h"
#include "material/materiallibrary.h"
#include "tree/terraindiffractionpath.h"
#include "tree/pathnode3d.h"
#include "tree/raypath3d.h"
#include "material/material.h"

#include "geometry/object2d/object2d.h"


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>


#include <CGAL/IO/OBJ.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>

//读取地形栅格所要用到的库
#include <gdal_priv.h>			
//读取地形物体所需要用到的库
#include <assimp/Importer.hpp>			
#include <assimp/scene.h>
#include <assimp/postprocess.h>




//cgal库用到的相关数据结构
typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
namespace SMS = CGAL::Surface_mesh_simplification;

class TerrainCell {
public:
	std::vector<TerrainSegment*> m_segments;		/** @brief	网格单元中包含的边 地址形式	*/
	std::vector<TerrainFacet*> m_facets;			/** @brief	网格单元中包含的面元 地址形式	*/
	Material* m_mat;								/** @brief	网格单元中的材质类型 地址形式	*/						
	Point2D m_cornerPoint;							/** @brief	网格单元左下角点坐标	*/
public:
	TerrainCell();
	TerrainCell(const TerrainCell& cell);			//拷贝构造函数
	~TerrainCell();
	bool GetIntersectEdgeFacet(Ray2D*& ray2d, Ray3DLite* ray3d, TerrainFacet* curFacet, TerrainFacet*& outFacet) const;			//获取cell中除输入边界面元外的其他边界面元,若找到面元，还需要更新ray2d的起始点坐标
};

//地形类型
class Terrain {
public:
	int m_terrainId;										/** @brief	地形ID	*/
	TERRAINCATEGORY m_category;								/** @brief	地形类型	*/
	std::vector<Point3D*> m_pointBuf;						/** @brief	存储地形上的所有坐标点	*/
	std::vector<TerrainFacet*> m_facetBuf;					/** @brief	存储地形上的所有面元	*/
	std::vector<TerrainSegment*> m_segmentBuf;				/** @brief	存储地形上的所有边	*/
	BBox3D m_bbox3d;										/** @brief	三维网格包围盒	*/
	PropagationProperty m_propagationProperty;				/** @brief	地形的电磁传播属性	*/
	RtLbsType m_averageRidgeGap;							/** @brief	平均峰峦间距值（用于区分地形）	*/
private:
	SurfaceMesh m_meshes;									/** @brief	网格面元数据,基于CGAL简化面元,可能会有多个mesh结构	*/
	unsigned m_voxelNum[2];									/** @brief	网格单元数量,x方向(cols),y方向(rows)	*/
	RtLbsType m_voxelExtent[2];								/** @brief	网格单元间隔,	x方向(cols),y方向(rows)*/
	RtLbsType m_voxelInvExtent[2];							/** @brief	网格单元间隔倒数	*/
	unsigned m_voxelCount;									/** @brief	网格单元总数	*/
	std::vector<TerrainCell*> m_gridCells;					/** @brief	地形网格单元数据	*/
	TERRAIN_CALCMODE m_calcMode;							/** @brief	地形求交判定计算模式	*/
	bool m_simplifyFlag;									/** @brief	地形是否简化标志	*/
	RtLbsType m_simplifyRate;								/** @brief	地形简化率	*/
	BBox2D m_bbox2d;										/** @brief	二维网格包围盒	*/
	Point3D m_centerPosition;								/** @brief	地形的质心坐标，加载地形时需要进行考虑计算	*/

public:
	Terrain();
	Terrain(const Terrain& terrain);						//拷贝构造函数
	~Terrain();
	bool Init(const TerrainConfig& config, const MaterialLibrary& matLibrary);																												//基于地形配置初始化地形						
	bool _simplify(double ratio);																																							//面元简化参数,ratio为简化的剩余百分比
	void write_OBJ(std::string& filename);																																					//将面元数据写入OBJ文件中
	void write_STL(std::string& filename);																																					//将面元数据写入STL文件中
	void write_GOCAD(std::string& filename);																																				//将面元数据写入到GOCAD中
	void write_PLY(std::string& filename);																																					//将面元数据写入到
	bool GetIntersect(Ray3DLite* rayInit, Point3D* intersectPoint = nullptr) const;																											//射线与地形相交无需求解具体交点（混合模式）
	bool IsBlock(const Point3D& ps, const Point3D& pe) const;																																//判定由起始点和终止点组成的路径是否被地形遮挡
	Material* GetMaterial(const Point3D& p) const;																																				//获取地形P点上的材质属性，仅存在地形栅格矩阵时可用
	bool IsValidPoint(const Point3D& p) const;																																				//判定点在地形中是否有效
	bool GetTerrainDiffractionPath(Point3D tx, Point3D rx, TerrainDiffractionPath*& outPath) const;																							//基于给定的收发点计算之间的绕射路径（若trx之间视距，则不给定绕射路径）
	bool GetTerrainReflectionPaths(const Point3D& tx, const Point3D& rx, std::vector<RayPath3D*>& outPath) const;																			//基于地形计算出一次反射路径
	void Update(const Vector3D& offset);																																					//基于位移更新地形
	void Update(const Euler& posture);																																						//基于姿态更新地形 以几何中心进行旋转
	void Update(const Vector3D& offset, const Euler& posture);																																//基于位移和姿态更新地形 以几何中心进行旋转
	RtLbsType GetObjectFoundationHeight(const Object2D* object) const;																														//获取二维物体在地形上的基础高度点

private:
	void _init();																																											/** @brief	初始化数据	*/
	bool _initData(float* elevation, std::vector<Material*>& matMatrix, int rows, int cols, RtLbsType rowGap, RtLbsType colGap, RtLbsType minValue, RtLbsType maxValue, RtLbsType ratio);					//初始化数据，高程矩阵
	bool _initData(const aiScene* scene, RtLbsType ratio, std::vector<Material*>& materials);																								//从obj文件中读取地形几何数据和材质属性数据
	void _release();																																										/** @brief	释放类所占用的堆内存	*/
	bool _transform(const SurfaceMesh& mesh, Material* mat = nullptr);																																//将surfacemesh转换为本地变量
	int _offset(int x, int y) const;																																						//根据输入的行列id确定在一维网格单元的索引值
	void _offset_reverse(unsigned voxelId, int& x, int& y) const;																															//计算反向偏移,由体素ID转换为二维索引值
	int _point2VoxelId(const Point2D& p, unsigned axis) const;																																//计算二维点在voxel中的坐标
	int _point2VoxelId(const Point3D& p, unsigned axis) const;																																//计算三维点在voxel中的坐标
	Point2D _voxelId2Point(int voxel[2]) const;																																				/** @brief	将voxelId转换为世界坐标	*/
	void _build(std::vector<Material*>& matMatrix);																																							//构建Terrain cell模型(栅格模型版本)
	void _build();																																											//构建Terrain cell模型(非孔洞或含孔洞版本)
	std::vector<unsigned> _getGridCoordAlongSegment(TerrainSegment* segment);																												//获取segment上的网格编号
	bool _getIntersect(Ray3DLite* ray, unsigned voxelId, Point3D* intersectPoint = nullptr) const;																							//射线与voxel求交， 栅格单元法专用
	bool _getIntersect(Ray3DLite* ray3d, Ray2D* ray2d, RtLbsType t, unsigned voxelId, unsigned& targetVoxelId) const;																		//射线与voxel求交，半边混合方法专用
	TerrainFacet* _getTerrainFacetViaPoint(const Point2D& point) const;																															//通过坐标点计算出在哪个面元内部-二维坐标点
	TerrainFacet* _getTerrainFacetViaPoint(const Point3D& point) const;																														//通过坐标点计算出在哪个面元内部-三维坐标点
	bool _getTerrainProfileFacets(const Point3D& txPosition, const Point3D& rxPosition, std::vector<TerrainFacet*>& outFacets) const;														//计算trx连线上的所经历的面元
};

#endif