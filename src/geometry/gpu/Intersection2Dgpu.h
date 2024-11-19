#ifndef RTLBS_INTERSECTION2DGPU
#define RTLBS_INTERSECTION2DGPU
#include "utility/enum.h"
#include "segment2dgpu.h"
#include "configuration/radiowave/propagation/propagationproperty.h"

class Point2D;
class Ray2DGPU;

class Intersection2DGPU {

public:
	bool m_isValid;											/** @brief	�ཻ״̬	*/
	Point2D m_intersect;									/** @brief	��������	*/
	PATHNODETYPE m_type;									/** @brief	�ڵ�����	*/
	RtLbsType m_ft;											/** @brief	����˾���ԭʼ�����ľ���	*/
	int m_matId;											/** @brief	�ڵ����ID	*/
	int m_segmentId;										/** @brief	����/͸�� �ж�����ԪId	*/
	int m_wedgeId;											/** @brief	�����ж�������Id	*/
	Ray2DGPU m_ray;											/** @brief	������Ϣ������	*/
	int m_prevId;											/** @brief	ǰһ���ڵ��Id	*/
	PropagationProperty m_propagationProperty;				/** @brief	����������Ԫ�Ĵ�������	*/

public:
	HOST_DEVICE_FUNC Intersection2DGPU();
	HOST_DEVICE_FUNC Intersection2DGPU(const Intersection2DGPU& intersect);				//��ֵ���캯��
	HOST_DEVICE_FUNC ~Intersection2DGPU();
	HOST_DEVICE_FUNC Intersection2DGPU& operator = (const Intersection2DGPU& intersect);
	HOST_DEVICE_FUNC bool IsCaptureRx(Point2D rx);
	HOST_DEVICE_FUNC Point2D GetVisualSource();  //�������Դ��λ��
	

};

#endif
