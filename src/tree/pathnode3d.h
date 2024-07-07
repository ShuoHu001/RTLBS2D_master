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



//�汾0-0-0-0 �ݲ����ǹ���Դ�ʹ�������
class PathNode3D {
public:
	unsigned m_depth;																												/** @brief	�ڵ����	*/
	PATHNODETYPE m_type;																											/** @brief	�ڵ�����	*/
	Material* m_mat;																												/** @brief	�ڵ����ڲ���	*/
	Point3D m_point;																												/** @brief	�ڵ�����	*/
	Point2D m_gs2D;																													/** @brief	ĩ�˹���Դ(��ά)	*/
	const Segment2D* m_primitive;																									/** @brief	�ڵ�������Ԫ��ָ��	*/
	const Wedge2D* m_wedge;																											/** @brief	�ڵ�����wedge��ָ��	*/
	const TerrainFacet* m_terrainFacet;																								/** @brief	�ڵ����ڵĵ�����Ԫ��ָ��	*/
public:
	PathNode3D();
	PathNode3D(const PathNode& node, RtLbsType h);																					//�ɶ�άPathNode�ڵ�ת��Ϊ��άPathNode�ڵ�Ĺ��캯��
	PathNode3D(const Point3D& point, PATHNODETYPE type, int depth = 0);																/** @brief	��ʼ�ڵ����ֹ�ڵ�Ĺ��캯��	*/
	PathNode3D(const Point3D& point, PATHNODETYPE type, const Segment2D* segment, int depth = 0);									/** @brief	����ڵ�Ĺ��캯��	*/
	PathNode3D(const Point3D& point, PATHNODETYPE type, const TerrainFacet* facet, int depth = 0);									/** @brief	���淴��ڵ�Ĺ��캯��	*/
	PathNode3D(const Point3D& point, PATHNODETYPE type, const Wedge2D* wedge, int depth = 0);										/** @brief	����ڵ�Ĺ��캯��	*/
	~PathNode3D();																													//��������
	void ConvertBy(const PathNode& node, RtLbsType h);																				//��PathNodeת��ΪPathNode3D
	void ConvertBy(const PathNodeGPU& node, RtLbsType h, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges);			//��PathNodeGPUת��ΪPathNode3D
	std::string ToString() const;																									//·���ڵ�ת��Ϊ�ַ���
};

#endif
