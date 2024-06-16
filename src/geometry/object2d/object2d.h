#ifndef RTLBS_OBJECT2D
#define RTLBS_OBJECT2D

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/segment2d.h"
#include "geometry/wedge2d.h"
#include "physical/propagationproperty.h"

class Object2D {
public:
	int m_objectId;									/** @brief	����ID	*/
	OBJECT2DCATEGORY m_category;					/** @brief	��������	*/
	int m_matId;									/** @brief	����ID	*/
	PropagationProperty m_propagationProperty;		/** @brief	��������	*/
	RtLbsType m_height;								/** @brief	����߶�	*/
	RtLbsType m_foundationHeight;					/** @brief	����ػ��߶�	*/
	std::vector<Segment2D*> m_segments;				/** @brief	���弸������	*/
	std::vector<Wedge2D*> m_wedges;					/** @brief	��������	*/

public:
	Object2D();
	~Object2D();
	void SetSegments(const std::vector<Segment2D*>& segments);		//�����߶�
	void InitWedges();												//��ʼ������
	bool IsContain(const Point2D& p) const;							//�Ƿ������ά��
	bool IsContain(const Point3D& p) const;							//�Ƿ������ά��
	RtLbsType GetObjectHeight() const;								//��ȡ����ĸ߶�(��ʵ�߶�)
};


#endif
