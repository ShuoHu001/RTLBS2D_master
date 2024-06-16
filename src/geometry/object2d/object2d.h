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
	int m_objectId;									/** @brief	物体ID	*/
	OBJECT2DCATEGORY m_category;					/** @brief	物体种类	*/
	int m_matId;									/** @brief	材质ID	*/
	PropagationProperty m_propagationProperty;		/** @brief	传播属性	*/
	RtLbsType m_height;								/** @brief	物体高度	*/
	RtLbsType m_foundationHeight;					/** @brief	物体地基高度	*/
	std::vector<Segment2D*> m_segments;				/** @brief	物体几何数据	*/
	std::vector<Wedge2D*> m_wedges;					/** @brief	棱劈数据	*/

public:
	Object2D();
	~Object2D();
	void SetSegments(const std::vector<Segment2D*>& segments);		//设置线段
	void InitWedges();												//初始化棱劈
	bool IsContain(const Point2D& p) const;							//是否包含二维点
	bool IsContain(const Point3D& p) const;							//是否包含三维点
	RtLbsType GetObjectHeight() const;								//获取物体的高度(真实高度)
};


#endif
