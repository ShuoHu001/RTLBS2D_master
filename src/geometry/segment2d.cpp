#include "segment2d.h"
#include "Intersection2D.h"
#include "wedge2d.h"

Segment2D::Segment2D()
	: m_ws(nullptr)
	, m_we(nullptr)
	, m_length(-FLT_MAX)
{
}

Segment2D::Segment2D(const Point2D& ps, const Point2D& pe, OBJECT2DCATEGORY category)
	: Primitive2D()
	, m_ps(ps)
	, m_pe(pe)
	, m_length(0.0)
	, m_ws(nullptr)
	, m_we(nullptr)
{
	//父类初始化
	m_objectCategory = category;
	Vector2D normal = pe - ps;
	m_dir = normal.Normalize();
	m_normal = Vector2D(m_dir.y, -m_dir.x, true);

	m_bbox.Union(m_ps);	//计算bbox
	m_bbox.Union(m_pe);
	m_bbox.m_isValid = true;

	m_length = (m_pe - m_ps).Length();	//计算线段长度

}

Segment2D::Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN)
	: Primitive2D()
	, m_ps(ps)
	, m_pe(pe)
	, m_ws(nullptr)
	, m_we(nullptr)
{
	m_refractN = refractN;//父类初始化
	m_id = globalId;
	Vector2D normal = pe - ps;
	m_dir = normal.Normalize();
	m_normal = Vector2D(m_dir.y, -m_dir.x, true);
	
	m_bbox.Union(m_ps);	//计算bbox
	m_bbox.Union(m_pe);
	m_bbox.m_isValid = true;
	
	m_length = (m_pe - m_ps).Length();	//计算线段长度
}

Segment2D::Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, RtLbsType refractNOut)
	: Primitive2D()
	, m_ps(ps)
	, m_pe(pe)
	, m_ws(nullptr)
	, m_we(nullptr)
{
	m_refractN = refractN;//父类初始化
	m_refractNOut = refractNOut;
	m_id = globalId;
	Vector2D normal = pe - ps;
	m_dir = normal.Normalize();
	m_normal = Vector2D(m_dir.y, -m_dir.x, true);

	m_bbox.Union(m_ps);//计算bbox
	m_bbox.Union(m_pe);
	m_bbox.m_isValid = true;
	
	m_length = (m_pe - m_ps).Length();//计算线段长度
}

Segment2D::Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, Wedge2D* ws, Wedge2D* we)
	: Primitive2D()
	, m_ps(ps)
	, m_pe(pe)
	, m_ws(ws)
	, m_we(we)
{
	m_refractN = refractN;//父类初始化
	m_id = globalId;

	Vector2D normal = m_pe - m_ps;
	m_normal = normal.Normalize();
	m_normal = Vector2D(m_dir.y, -m_dir.x, true);
	
	m_bbox.Union(m_ps);	//计算bbox
	m_bbox.Union(m_pe);
	m_bbox.m_isValid = true;

	m_length = (m_pe - m_ps).Length();	//计算线段长度
}

Segment2D::Segment2D(Point2D ps, Point2D pe, int globalId, RtLbsType refractN, RtLbsType refractNOut, Wedge2D* ws, Wedge2D* we)
	: Primitive2D()
	, m_ps(ps)
	, m_pe(pe)
	, m_ws(ws)
	, m_we(we)
{
	m_refractN = refractN;//父类初始化
	m_refractNOut = refractNOut;
	m_id = globalId;

	Vector2D normal = m_pe - m_ps;
	m_normal = normal.Normalize();
	m_normal = Vector2D(m_dir.y, -m_dir.x, true);

	m_bbox.Union(m_ps);	//计算bbox
	m_bbox.Union(m_pe);
	m_bbox.m_isValid = true;

	m_length = (m_pe - m_ps).Length();	//计算线段长度
}

Segment2D::Segment2D(const Segment2D* segment)
{
	m_ps = segment->m_ps;
	m_pe = segment->m_pe;
	m_normal = segment->m_normal;
	m_dir = segment->m_dir;
	
	m_ws = segment->m_ws;
	m_we = segment->m_we;
	m_length = segment->m_length;
	//父类赋值
	m_bbox = segment->m_bbox;
	m_id = segment->m_id;
	m_objectId = segment->m_objectId;
	m_matId = segment->m_matId;
	m_refractN = segment->m_refractN;
	m_refractNOut = segment->m_refractNOut;

}

Segment2D::~Segment2D()
{
}

bool Segment2D::operator==(const Segment2D& other) const
{
	if (m_id != other.m_id)
		return false;
	return true;
}

bool Segment2D::operator!=(const Segment2D& other) const
{
	return !(*this == other);
}

Segment2D& Segment2D::operator=(const Segment2D& segment)
{
	m_ps = segment.m_ps;
	m_pe = segment.m_pe;
	m_normal = segment.m_normal;
	m_dir = segment.m_dir;
	m_ws = segment.m_ws;
	m_we = segment.m_we;
	m_length = segment.m_length;
	//父类赋值
	m_id = segment.m_id;
	m_objectId = segment.m_objectId;
	m_matId = segment.m_matId;
	m_refractN = segment.m_refractN;
	m_refractNOut = segment.m_refractNOut;
	m_propagationProperty = segment.m_propagationProperty;
	m_bbox = segment.m_bbox;
	return (*this);
}

RtLbsType Segment2D::DistanceToPoint(const Point2D& p) const
{
	Vector2D sp = p - m_ps;							/** @brief	线段起点到P点的向量	*/
	Vector2D ep = p - m_pe;							/** @brief	线段终点到P点的向量	*/
	RtLbsType spLen = sp.Length();					/** @brief	线段起点到P点的距离	*/
	RtLbsType epLen = ep.Length();					/** @brief	线段终点到P点的距离	*/

	//计算sp在线段方向上的投影长度
	RtLbsType spProjLen = sp * m_dir;							/** @brief	SP沿线段方向的投影长度	*/
	if (spProjLen >= 0 && spProjLen <= m_length) {				//当且仅当投影长度满足范围时返回点到线段中的距离
		Point2D projPoint = m_ps + m_dir * spProjLen;			/** @brief	投影点坐标	*/
		Vector2D sprojP = projPoint - m_ps;						/** @brief	起始点指向投影点的向量	*/
		Vector2D projPP = p - projPoint;						/** @brief	投影点到点P的向量	*/
		return projPP.Length();
	}
	return std::min(spLen, epLen);								//一般情况下返回P点到起始点和终止点的最小值即为点到线段的最小距离
}

bool Segment2D::IsInSegment(const Point2D& p) const
{
	if (DistanceToPoint(p) < EPSILON) {
		return true;
	}
	return false;
}

RtLbsType Segment2D::GetCenter(int axis) const
{
	return (m_ps[axis] + m_pe[axis]) * 0.5;
}


bool Segment2D::GetIntersect(const Ray2D& ray, Intersection2D* intersect)
{
	RtLbsType tmax;
	RtLbsType tmin = m_bbox.Intersect(ray, &tmax);
	if (tmin < 0.0) {//不相交
		return false;
	}
	RtLbsType denominator = Cross(ray.m_Dir, m_dir);
	if (abs(denominator) < EPSILON) { //射线与面元共线情况
		if (abs(Cross(ray.m_Dir, m_ps - ray.m_Ori)) < EPSILON) {//当前射线与面元共线,最大只有一个绕射点
			if ((ray.m_Ori - m_ps) * (ray.m_Ori - m_pe) <= 0)//排除射线起点在面元上
				return false;
			if (intersect) {
				intersect->m_ft = tmin;//更新交点距离源点的距离
				intersect->m_segment = this;//绕射加入面元赋值，方便判断
				intersect->m_propagationProperty = m_propagationProperty;			//传播属性赋值
				if (Dot(ray.m_Dir, m_dir) < 0) {//射线方向与面元方向相反,交点为线段终点
					intersect->m_intersect = m_pe;
					if (m_ws) {//若m_ws有值,则代表绕射点真实存在
						intersect->m_type = NODE_DIFF;
						intersect->m_wedges.push_back(m_ws);
						return true;
					}
				}
				else {
					intersect->m_intersect = m_ps;
					if (m_we) {
						intersect->m_type = NODE_DIFF;
						intersect->m_wedges.push_back(m_we);
						return true;
					}
				}
				//反射节点
				intersect->m_segment = this;
				intersect->m_type = NODE_REFL;
				return true;
			}
		}
		return false;
	}
	Vector2D p = m_ps - ray.m_Ori;
	RtLbsType t = static_cast<RtLbsType>(Cross(p, m_dir) / denominator /*- EPSILON*/); //t:交点距离面元起点的距离,减小t的值有利于规避计算误差极限
	RtLbsType u = static_cast<RtLbsType>(Cross(p, ray.m_Dir) / denominator/(m_pe-m_ps).Length()); //交点在线段上的百分比
	if (abs(u) < EPSILON) u = static_cast<RtLbsType>(0.0);
	if (abs(u - static_cast <RtLbsType>(1.0)) < EPSILON) u = static_cast<RtLbsType>(1.0);
	if (t >= EPSILON && u >= static_cast <RtLbsType>(0.0) && u <= static_cast <RtLbsType>(1.0)) {//这里考虑t>EPSILON指明不判定起点在面元上的射线相交,并附加考虑误差
		if (intersect) {
			intersect->m_intersect = ray.m_Ori + ray.m_Dir * t;
			intersect->m_ft = t;
			intersect->m_segment = this;
			intersect->m_propagationProperty = m_propagationProperty;			//传播属性赋值
			intersect->m_type = NODE_REFL;//反射节点
			intersect->m_u = u;
			if (u == static_cast <RtLbsType>(0.0) && m_ws !=nullptr) {
				intersect->m_type = NODE_DIFF;//绕射节点
				intersect->m_wedges.push_back(m_ws);
				return true;
			}
			if (u == static_cast <RtLbsType>(1.0) && m_we != nullptr) {
				intersect->m_type = NODE_DIFF;//绕射节点
				intersect->m_wedges.push_back(m_we);
				return true;
			}
		}
		return true;
	}
	return false;
}

bool Segment2D::GetIntersectNoBBox(const Ray2D& ray, Intersection2D* intersect)
{
	RtLbsType denominator = Cross(ray.m_Dir, m_dir);
	if (abs(denominator) < EPSILON) { //射线与面元共线情况
		if (abs(Cross(ray.m_Dir, m_ps - ray.m_Ori)) < EPSILON) {//当前射线与面元共线,最大只有一个绕射点
			if ((ray.m_Ori - m_ps) * (ray.m_Ori - m_pe) <= 0)//排除射线起点在面元上
				return false;
			if (intersect) {
				RtLbsType tmax;
				RtLbsType tmin = m_bbox.Intersect(ray, &tmax);
				if (tmin < 0.0) {//不相交
					return false;
				}
				intersect->m_ft = tmin;//更新交点距离源点的距离
				intersect->m_segment = this;//绕射加入面元赋值，方便判断
				intersect->m_propagationProperty = m_propagationProperty;			//传播属性赋值
				if (Dot(ray.m_Dir, m_dir) < 0) {//射线方向与面元方向相反,交点为线段终点
					intersect->m_intersect = m_pe;
					if (m_ws) {//若m_ws有值,则代表绕射点真实存在
						intersect->m_type = NODE_DIFF;
						intersect->m_wedges.push_back(m_ws);
						return true;
					}
				}
				else {
					intersect->m_intersect = m_ps;
					if (m_we) {
						intersect->m_type = NODE_DIFF;
						intersect->m_wedges.push_back(m_we);
						return true;
					}
				}
				//反射节点
				intersect->m_segment = this;
				intersect->m_type = NODE_REFL;
				return true;
			}
		}
		return false;
	}
	Vector2D p = m_ps - ray.m_Ori;
	RtLbsType t = static_cast<RtLbsType>(Cross(p, m_dir) / denominator /*- EPSILON*/); //t:交点距离面元起点的距离,减小t的值有利于规避计算误差极限
	RtLbsType u = static_cast<RtLbsType>(Cross(p, ray.m_Dir) / denominator / (m_pe - m_ps).Length()); //交点在线段上的百分比
	if (abs(u) < EPSILON) u = static_cast<RtLbsType>(0.0);
	if (abs(u - static_cast <RtLbsType>(1.0)) < EPSILON) u = static_cast<RtLbsType>(1.0);
	if (t >= EPSILON && u >= static_cast <RtLbsType>(0.0) && u <= static_cast <RtLbsType>(1.0)) {//这里考虑t>EPSILON指明不判定起点在面元上的射线相交,并附加考虑误差
		if (intersect) {
			intersect->m_intersect = ray.m_Ori + ray.m_Dir * t;
			intersect->m_ft = t;
			intersect->m_segment = this;
			intersect->m_type = NODE_REFL;//反射节点
			intersect->m_u = u;
			intersect->m_propagationProperty = m_propagationProperty;			//传播属性赋值
			if (u == static_cast <RtLbsType>(0.0) && m_ws != nullptr) {
				intersect->m_type = NODE_DIFF;//绕射节点
				intersect->m_wedges.push_back(m_ws);
				return true;
			}
			if (u == static_cast <RtLbsType>(1.0) && m_we != nullptr) {
				intersect->m_type = NODE_DIFF;//绕射节点
				intersect->m_wedges.push_back(m_we);
				return true;
			}
		}
		return true;
	}
	return false;
}

bool Segment2D::GetIntersect(const Segment2D& segment, Intersection2D* intersect)
{
	//构造基本射线
	Ray2D ray(segment.m_ps, segment.m_dir);
	Intersection2D newIntersect;
	if (!GetIntersectNoBBox(ray, &newIntersect)) {						//若射线与不相交，则返回false
		return false;
	}
	RtLbsType distanceDiff = newIntersect.m_ft - segment.m_length;		/** @brief	交点距离与线段长度的距离差	*/
	if (distanceDiff > EPSILON) {										//若交点距离线段起点的长度大于线段长度，则表明线段与线段不相交,返回false
		return false;
	}
	if (intersect != nullptr) {
		*intersect = newIntersect;
	}
	return true;
}

const BBox2D& Segment2D::GetBBox() const
{
    return m_bbox;
}

Segment2DGPU Segment2D::Convert2GPU()
{
	Segment2DGPU segmentGPU;
	segmentGPU.m_ps = m_ps;
	segmentGPU.m_pe = m_pe;
	segmentGPU.m_normal = m_normal;
	segmentGPU.m_dir = m_dir;
	segmentGPU.m_length = m_length;
	segmentGPU.m_bbox = m_bbox.Convert2GPU();
	segmentGPU.m_primitive_id = m_id;
	segmentGPU.m_refractN = m_refractN;
	segmentGPU.m_refractNOut = m_refractNOut;
	segmentGPU.m_propagationProperty = m_propagationProperty;
	return segmentGPU;
}

void Segment2D::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
    writer.StartObject();
    writer.String("m_ps"); m_ps.Serialize(writer);
    writer.String("m_pe"); m_pe.Serialize(writer);
    writer.String("m_normal"); m_normal.Serialize(writer);
    writer.EndObject();
}

bool Segment2D::Deserialize(const rapidjson::Value& value)
{
    if (value.IsObject()) {
        const rapidjson::Value& m_psValue = value["m_ps"];
        const rapidjson::Value& m_peValue = value["m_pe"];
        const rapidjson::Value& m_normalValue = value["m_normal"];
        return m_ps.Deserialize(m_psValue) && m_pe.Deserialize(m_peValue) && m_normal.Deserialize(m_normalValue);

    }
    return false;
}
