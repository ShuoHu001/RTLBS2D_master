#ifndef RTLBS_LBSTREENODE
#define RTLBS_LBSTREENODE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "geometry/ray2d.h"
#include "geometry/segment2d.h"
#include "geometry/wedge2d.h"
#include "localization/generalsource.h"
#include "radiowave/raypath/raytreenode.h"
#include "radiowave/raypath/gpu/treenodegpu.h"
#include "equipment/sensor/sensordata.h"


class LBSTreeNode {
public:
	PATHNODETYPE m_type;											/** @brief	�ڵ�����	*/
	uint16_t m_depth;												/** @brief	�ڵ����ڵ����	*/
	Point2D m_position;												/** @brief	�ڵ���������	*/
	Point2D m_sourcePosition;										/** @brief	����Դ�����ڵ�����	*/
	Vector2D m_rayDir;												/** @brief	�ڵ��������ߵķ�������	*/
	SensorData* m_sensorData;										/** @brief	����������	*/
	Segment2D* m_segment;											/** @brief	�ڵ�������Ԫ	*/
	Wedge2D* m_wedge;												/** @brief	�ڵ���������	*/
	PathNode m_originPathNode;										/** @brief	��pathnode��Դ�ı���	*/

public:
	LBSTreeNode();
	LBSTreeNode(const LBSTreeNode& node);
	LBSTreeNode(const PathNode& node, SensorData* sensorData);													//��·���ڵ���г�ʼ������AOA�͵Ķ�λ�㷨
	LBSTreeNode(const PathNode& node);																			//��·���ڵ���г�ʼ����ֻ�ʺ�TDOA�Ķ�λ�㷨
	LBSTreeNode(const TreeNodeGPU& node, Segment2D* segment, Wedge2D* wedge, SensorData* sensorData);			//��GPU���ڵ���г�ʼ��-����
	~LBSTreeNode();
	LBSTreeNode& operator = (const LBSTreeNode& node);

	void GetGeneralSource_AOA(GeneralSource* source) const;														//��ȡ����Դ-AOA�ͷ����������� AOA��AOA-TDOA
	void GetGeneralSource_TOA(GeneralSource* source) const;														//��ȡ����Դ-TOA����, ����TOA
	void GetGeneralSource_TDOA(GeneralSource* source) const;													//��ȡ����Դ-TDOA�ͷ�����������TDOA
};

#endif
