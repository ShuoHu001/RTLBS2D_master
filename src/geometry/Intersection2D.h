#ifndef RTLBS_INTERSECTION2D
#define RTLBS_INTERSECTION2D
#include "utility/enum.h"
#include "wedge2d.h"
#include "ray2d.h"
#include "geometry/gpu/Intersection2Dgpu.h"
#include "physical/propagationproperty.h"

class Point2D;
class Segment2D;

class Intersection2D {

public:
	Point2D m_intersect;									/** @brief	��������	*/
	PATHNODETYPE m_type;									/** @brief	�ڵ�����	*/
	RtLbsType m_ft;											/** @brief	����˾���ԭʼ�����ľ���	*/
	Segment2D* m_segment;									/** @brief	����/͸�� �ж�����Ԫ	*/
	std::vector<Wedge2D*> m_wedges;							/** @brief	�����ж���Ш�ν�,����ǻ��ж�������	*/
	RtLbsType m_u;											/** @brief	�߶εķָ����,������������λ��	*/
	PropagationProperty m_propagationProperty;				/** @brief	��������-����ȷ����������	*/

public:
	Intersection2D();
	Intersection2D(Point2D intersect, PATHNODETYPE type, RtLbsType t, Segment2D* segment); //����ڵ�����
	Intersection2D(Point2D intersect, PATHNODETYPE type, RtLbsType t);						//�Ӿ�ڵ�����
	~Intersection2D();
	bool Update(Ray2D& ray); //���½�����Ϣ,�����ཻ�ж�ֻ�Ǽ��ν������Ҫ���ݽ�һ��������Ϣ�����ཻ��Ϣ

	Intersection2DGPU Convert2GPU();
	
private:
	bool ValidWedges(Ray2D& ray);

};

#endif
