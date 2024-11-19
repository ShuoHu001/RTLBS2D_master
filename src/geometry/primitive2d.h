#ifndef RTLBS_PRIMITIVE2D
#define RTLBS_PRIMITIVE2D

#include "bbox2d.h"
#include "material/material.h"
#include "utility/enum.h"
#include "configuration/radiowave/propagation/propagationproperty.h"


class Intersection2D;
class Primitive2D{

public:
	int m_id;										/** @brief	ȫ��ID��ȫ��Ψһ	*/
	int m_objectId;									/** @brief	��Ԫ����������	*/
	OBJECT2DCATEGORY m_objectCategory;				/** @brief	��Դ�������������	*/
	Material* m_mat;								/** @brief	���ʲ���	*/
	RtLbsType m_refractN;							/** @brief	����������,���ʱ����������	*/
	RtLbsType m_refractNOut;						/** @brief	���ʷ���������Ľ��ʵ�������	*/
	PropagationProperty m_propagationProperty;		/** @brief	��Ԫ�ϵĴ�������	*/
	mutable BBox2D m_bbox;							/** @brief	��Χ��	*/


public:
	Primitive2D();
	Primitive2D(int id, int matId, RtLbsType refractN);
	Primitive2D(int id, int matId, RtLbsType refractN, RtLbsType refractNOut);
	virtual ~Primitive2D(){}

public:
	virtual bool GetIntersect(const Ray2D& r, Intersection2D* intersect) = 0;
	virtual const BBox2D& GetBBox() const = 0;
	virtual void CLearBBoxCache();
	void SetID(int id) { m_id = id; }
	int64_t GetID() const { return m_id; }
};

#endif
