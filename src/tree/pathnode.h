#ifndef RTLBS_PATHNODE
#define RTLBS_PATHNODE


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/Intersection2D.h"
#include "physical/limitinfo.h"

class Point2D;
class Segment2D;
class PathNode3D;

class PathNode {

public:
	
	LimitInfo m_limitInfo;/** @brief	节点限制信息	*/
	PATHNODETYPE m_type;/** @brief	节点类型	*/
	Point2D m_point;/** @brief	节点坐标	*/
	Segment2D* m_segment;/** @brief	节点所在面元的指针	*/
	Wedge2D* m_wedge;/** @brief	节点所在wedge的指针	*/
	Ray2D m_prevRay; /** @brief	节点之前的射线(引用变量)	*/
	Ray2D m_nextRay; /** @brief	节点之后的射线(引用变量)	*/
	bool m_bTraceBack; /** @brief	是否为可回溯节点，即利用广义源进行修正路径，包含有透射节点的禁止通过广义源修正	*/
	
	Point2D m_source;	/** @brief	广义源点的坐标	*/
	RtLbsType m_ft;		/** @brief	当前节点距离root源的距离	*/  

public:
	PathNode();//统一跟正，所有的pathnode存储为前一个节点的ray
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point);															//根节点的初始化
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, const Ray2D& prevRay);											//视距节点的初始化/终止节点的初始化
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay);					//反射终止节点的初始化
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Segment2D* primitive, const Ray2D& prevRay, const Ray2D& nextRay); //反射节点的初始化
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay, const Ray2D& nextRay);		//绕射节点的初始化
	PathNode(const LimitInfo& limitInfo, PATHNODETYPE type, Point2D point, Wedge2D* wedge, const Ray2D& prevRay);							//绕射终止节点的初始化
	PathNode(const PathNode& pathnode);         //复制节点
	~PathNode() {};
	PathNode& operator = (PathNode& node);//赋值运算符重载
	bool IsContainPointByAngle(Point2D p); //当前节点是否在角度域内捕获某个坐标
};

#endif
