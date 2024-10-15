#ifndef RTLBS_PRIMITIVE2D
#define RTLBS_PRIMITIVE2D

#include "bbox2d.h"
#include "material/material.h"
#include "utility/enum.h"
#include "configuration/radiowave/propagation/propagationproperty.h"


class Intersection2D;
class Primitive2D{

public:
	int m_id;										/** @brief	全局ID，全局唯一	*/
	int m_objectId;									/** @brief	面元所属物体编号	*/
	OBJECT2DCATEGORY m_objectCategory;				/** @brief	面源所在物体的种类	*/
	Material* m_mat;								/** @brief	介质材料	*/
	RtLbsType m_refractN;							/** @brief	介质折射率,介质本身的折射率	*/
	RtLbsType m_refractNOut;						/** @brief	介质法向量朝外的介质的折射率	*/
	PropagationProperty m_propagationProperty;		/** @brief	面元上的传播属性	*/
	mutable BBox2D m_bbox;							/** @brief	包围盒	*/


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
