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

//��ȡ����դ����Ҫ�õ��Ŀ�
#include <gdal_priv.h>			
//��ȡ������������Ҫ�õ��Ŀ�
#include <assimp/Importer.hpp>			
#include <assimp/scene.h>
#include <assimp/postprocess.h>




//cgal���õ���������ݽṹ
typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
namespace SMS = CGAL::Surface_mesh_simplification;

class TerrainCell {
public:
	std::vector<TerrainSegment*> m_segments;		/** @brief	����Ԫ�а����ı� ��ַ��ʽ	*/
	std::vector<TerrainFacet*> m_facets;			/** @brief	����Ԫ�а�������Ԫ ��ַ��ʽ	*/
	Material* m_mat;								/** @brief	����Ԫ�еĲ������� ��ַ��ʽ	*/						
	Point2D m_cornerPoint;							/** @brief	����Ԫ���½ǵ�����	*/
public:
	TerrainCell();
	TerrainCell(const TerrainCell& cell);			//�������캯��
	~TerrainCell();
	bool GetIntersectEdgeFacet(Ray2D*& ray2d, Ray3DLite* ray3d, TerrainFacet* curFacet, TerrainFacet*& outFacet) const;			//��ȡcell�г�����߽���Ԫ��������߽���Ԫ,���ҵ���Ԫ������Ҫ����ray2d����ʼ������
};

//��������
class Terrain {
public:
	int m_terrainId;										/** @brief	����ID	*/
	TERRAINCATEGORY m_category;								/** @brief	��������	*/
	std::vector<Point3D*> m_pointBuf;						/** @brief	�洢�����ϵ����������	*/
	std::vector<TerrainFacet*> m_facetBuf;					/** @brief	�洢�����ϵ�������Ԫ	*/
	std::vector<TerrainSegment*> m_segmentBuf;				/** @brief	�洢�����ϵ����б�	*/
	BBox3D m_bbox3d;										/** @brief	��ά�����Χ��	*/
	PropagationProperty m_propagationProperty;				/** @brief	���εĵ�Ŵ�������	*/
	RtLbsType m_averageRidgeGap;							/** @brief	ƽ�����ͼ��ֵ���������ֵ��Σ�	*/
private:
	SurfaceMesh m_meshes;									/** @brief	������Ԫ����,����CGAL����Ԫ,���ܻ��ж��mesh�ṹ	*/
	unsigned m_voxelNum[2];									/** @brief	����Ԫ����,x����(cols),y����(rows)	*/
	RtLbsType m_voxelExtent[2];								/** @brief	����Ԫ���,	x����(cols),y����(rows)*/
	RtLbsType m_voxelInvExtent[2];							/** @brief	����Ԫ�������	*/
	unsigned m_voxelCount;									/** @brief	����Ԫ����	*/
	std::vector<TerrainCell*> m_gridCells;					/** @brief	��������Ԫ����	*/
	TERRAIN_CALCMODE m_calcMode;							/** @brief	�������ж�����ģʽ	*/
	bool m_simplifyFlag;									/** @brief	�����Ƿ�򻯱�־	*/
	RtLbsType m_simplifyRate;								/** @brief	���μ���	*/
	BBox2D m_bbox2d;										/** @brief	��ά�����Χ��	*/
	Point3D m_centerPosition;								/** @brief	���ε��������꣬���ص���ʱ��Ҫ���п��Ǽ���	*/

public:
	Terrain();
	Terrain(const Terrain& terrain);						//�������캯��
	~Terrain();
	bool Init(const TerrainConfig& config, const MaterialLibrary& matLibrary);																												//���ڵ������ó�ʼ������						
	bool _simplify(double ratio);																																							//��Ԫ�򻯲���,ratioΪ�򻯵�ʣ��ٷֱ�
	void write_OBJ(std::string& filename);																																					//����Ԫ����д��OBJ�ļ���
	void write_STL(std::string& filename);																																					//����Ԫ����д��STL�ļ���
	void write_GOCAD(std::string& filename);																																				//����Ԫ����д�뵽GOCAD��
	void write_PLY(std::string& filename);																																					//����Ԫ����д�뵽
	bool GetIntersect(Ray3DLite* rayInit, Point3D* intersectPoint = nullptr) const;																											//����������ཻ���������彻�㣨���ģʽ��
	bool IsBlock(const Point3D& ps, const Point3D& pe) const;																																//�ж�����ʼ�����ֹ����ɵ�·���Ƿ񱻵����ڵ�
	Material* GetMaterial(const Point3D& p) const;																																				//��ȡ����P���ϵĲ������ԣ������ڵ���դ�����ʱ����
	bool IsValidPoint(const Point3D& p) const;																																				//�ж����ڵ������Ƿ���Ч
	bool GetTerrainDiffractionPath(Point3D tx, Point3D rx, TerrainDiffractionPath*& outPath) const;																							//���ڸ������շ������֮�������·������trx֮���Ӿ࣬�򲻸�������·����
	bool GetTerrainReflectionPaths(const Point3D& tx, const Point3D& rx, std::vector<RayPath3D*>& outPath) const;																			//���ڵ��μ����һ�η���·��
	void Update(const Vector3D& offset);																																					//����λ�Ƹ��µ���
	void Update(const Euler& posture);																																						//������̬���µ��� �Լ������Ľ�����ת
	void Update(const Vector3D& offset, const Euler& posture);																																//����λ�ƺ���̬���µ��� �Լ������Ľ�����ת
	RtLbsType GetObjectFoundationHeight(const Object2D* object) const;																														//��ȡ��ά�����ڵ����ϵĻ����߶ȵ�

private:
	void _init();																																											/** @brief	��ʼ������	*/
	bool _initData(float* elevation, std::vector<Material*>& matMatrix, int rows, int cols, RtLbsType rowGap, RtLbsType colGap, RtLbsType minValue, RtLbsType maxValue, RtLbsType ratio);					//��ʼ�����ݣ��߳̾���
	bool _initData(const aiScene* scene, RtLbsType ratio, std::vector<Material*>& materials);																								//��obj�ļ��ж�ȡ���μ������ݺͲ�����������
	void _release();																																										/** @brief	�ͷ�����ռ�õĶ��ڴ�	*/
	bool _transform(const SurfaceMesh& mesh, Material* mat = nullptr);																																//��surfacemeshת��Ϊ���ر���
	int _offset(int x, int y) const;																																						//�������������idȷ����һά����Ԫ������ֵ
	void _offset_reverse(unsigned voxelId, int& x, int& y) const;																															//���㷴��ƫ��,������IDת��Ϊ��ά����ֵ
	int _point2VoxelId(const Point2D& p, unsigned axis) const;																																//�����ά����voxel�е�����
	int _point2VoxelId(const Point3D& p, unsigned axis) const;																																//������ά����voxel�е�����
	Point2D _voxelId2Point(int voxel[2]) const;																																				/** @brief	��voxelIdת��Ϊ��������	*/
	void _build(std::vector<Material*>& matMatrix);																																							//����Terrain cellģ��(դ��ģ�Ͱ汾)
	void _build();																																											//����Terrain cellģ��(�ǿ׶��򺬿׶��汾)
	std::vector<unsigned> _getGridCoordAlongSegment(TerrainSegment* segment);																												//��ȡsegment�ϵ�������
	bool _getIntersect(Ray3DLite* ray, unsigned voxelId, Point3D* intersectPoint = nullptr) const;																							//������voxel�󽻣� դ��Ԫ��ר��
	bool _getIntersect(Ray3DLite* ray3d, Ray2D* ray2d, RtLbsType t, unsigned voxelId, unsigned& targetVoxelId) const;																		//������voxel�󽻣���߻�Ϸ���ר��
	TerrainFacet* _getTerrainFacetViaPoint(const Point2D& point) const;																															//ͨ��������������ĸ���Ԫ�ڲ�-��ά�����
	TerrainFacet* _getTerrainFacetViaPoint(const Point3D& point) const;																														//ͨ��������������ĸ���Ԫ�ڲ�-��ά�����
	bool _getTerrainProfileFacets(const Point3D& txPosition, const Point3D& rxPosition, std::vector<TerrainFacet*>& outFacets) const;														//����trx�����ϵ�����������Ԫ
};

#endif