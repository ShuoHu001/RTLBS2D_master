#ifndef RTLBS_TREENODEGPU
#define RTLBS_TREENODEGPU

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/gpu/ray2dgpu.h"

class TreeNodeGPU {
public:
	bool m_isValid;											/** @brief	�Ƿ���Ч��־	*/
	PATHNODETYPE m_type;									/** @brief	�ڵ�����	*/
	uint16_t m_depth;										/** @brief	�ڵ����	*/
	RtLbsType m_t;											/** @brief	��������	*/
	Point2D m_point;										/** @brief	�ڵ�����	*/
	int m_segmentId;										/** @brief	����/͸�� �ж�����ԪId	*/
	int m_wedgeId;											/** @brief	�����ж�������Id	*/
	Ray2DGPU m_nextRay;										/** @brief	�ɽڵ�����������	*/

public:
	HOST_DEVICE_FUNC TreeNodeGPU();												//���湹�캯��
	HOST_DEVICE_FUNC TreeNodeGPU(const TreeNodeGPU& node);						//��ֵ���캯��
	HOST_DEVICE_FUNC ~TreeNodeGPU();											//��������
	HOST_DEVICE_FUNC TreeNodeGPU operator = (const TreeNodeGPU& node);			//=�������
	HOST_DEVICE_FUNC Point2D GetGeneralSource();								//�������Դ
};


#endif
