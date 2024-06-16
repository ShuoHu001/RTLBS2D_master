#include "terrainfacet.h"
#include "terrainsegment.h"

TerrainFacet::TerrainFacet()
	: m_facetId(-1)
	, m_isEdgeFacet(false)
	, m_p1(nullptr)
	, m_p2(nullptr)
	, m_p3(nullptr)
	, m_segment1(nullptr)
	, m_segment2(nullptr)
	, m_segment3(nullptr)
	, m_matId(-1)
{
}

TerrainFacet::TerrainFacet(int facetId, Point3D* p1, Point3D* p2, Point3D* p3, int matId)
	: m_facetId(facetId)
	, m_isEdgeFacet(false)
	, m_p1(p1)
	, m_p2(p2)
	, m_p3(p3)
	, m_segment1(nullptr)
	, m_segment2(nullptr)
	, m_segment3(nullptr)
	, m_matId(matId)
{
	m_bbox.Union(m_p1);
	m_bbox.Union(m_p2);
	m_bbox.Union(m_p3);
	//计算法向量，默认p1,p2,p3为逆时针方向
	m_normal = Cross((*m_p2 - *m_p1), (*m_p3 - *m_p1)).Normalize();
}

TerrainFacet::TerrainFacet(int facetId, Point3D* p1, Point3D* p2, Point3D* p3, TerrainSegment* segment1, TerrainSegment* segment2, TerrainSegment* segment3, int matId)
	: m_facetId(facetId)
	, m_isEdgeFacet(false)
	, m_p1(p1)
	, m_p2(p2)
	, m_p3(p3)
	, m_segment1(segment1)
	, m_segment2(segment2)
	, m_segment3(segment3)
	, m_matId(matId)
{
	m_bbox.Union(m_p1);
	m_bbox.Union(m_p2);
	m_bbox.Union(m_p3);
	//计算法向量，默认p1,p2,p3为逆时针方向
	m_normal = Cross((*m_p2 - *m_p1), (*m_p3 - *m_p1)).Normalize();

}

TerrainFacet::TerrainFacet(const TerrainFacet& tf)
    : m_facetId(tf.m_facetId)
	, m_isEdgeFacet(tf.m_isEdgeFacet)
    , m_p1(tf.m_p1)
    , m_p2(tf.m_p2)
    , m_p3(tf.m_p3)
	, m_normal(tf.m_normal)
    , m_segment1(tf.m_segment1)
    , m_segment2(tf.m_segment2)
    , m_segment3(tf.m_segment3)
	, m_bbox(tf.m_bbox)
	, m_matId(tf.m_matId)
{
}

TerrainFacet::~TerrainFacet()
{
}

TerrainFacet TerrainFacet::operator=(const TerrainFacet& tf)
{
    m_facetId = tf.m_facetId;
	m_isEdgeFacet = tf.m_isEdgeFacet;
    m_p1 = tf.m_p1;
    m_p2 = tf.m_p2;
    m_p3 = tf.m_p3;
	m_normal = tf.m_normal;
    m_segment1 = tf.m_segment1;
    m_segment2 = tf.m_segment2;
    m_segment3 = tf.m_segment3;
	m_bbox = tf.m_bbox;
	m_matId = tf.m_matId;
    return *this;
}

bool TerrainFacet::operator==(const TerrainFacet& other) const
{
	if (m_facetId == other.m_facetId)
		return true;
    return false;
}

bool TerrainFacet::operator!=(const TerrainFacet& other) const
{
	return !(*this == other);
}

void TerrainFacet::AssignEdge(TerrainSegment* segment)
{
	if (!m_segment1) {
		m_segment1 = segment;
		return;
	}
	if (!m_segment2) {
		m_segment2 = segment;
		return;
	}
	if (!m_segment3) {
		m_segment3 = segment;
		return;
	}
	LOG_WARNING << "superfluous edge assignment." << ENDL;
}

bool TerrainFacet::GetIntersect(Ray3DLite* ray, Point3D* point) const
{
	//去除射线基点所在的面元
	if (ray->m_facetId == m_facetId)
		return false;
	Vector3D e1 = *m_p2 - *m_p1;
	Vector3D e2 = *m_p3 - *m_p1;
	Vector3D s1 = ray->m_dir.Cross(e2);
	double divisor = s1 * e1;
	if (fabs(divisor) < EPSILON) {
		return false; // This ray is parallel to this triangle.
	}
	double invDivisor = 1.0 / divisor;

	Vector3D d = ray->m_ori - *m_p1;
	double u = d * s1 * invDivisor;
	if (u < -EPSILON || u > 1.0 + EPSILON) { //u between (-EPSILON,1+EPSILON)
		return false;
	}
	Vector3D s2 = d.Cross(e1);
	double v = ray->m_dir * s2 * invDivisor;
	if (v < -EPSILON || u + v > 1.0 + EPSILON) {
		return false;
	}
	double t = e2 * s2 * invDivisor;
	if (t < 0.0) {
		return false;
	}
	if (point) {				//若交点信息给定，则进行交点信息的赋值（交点赋值）
		*point = ray->m_ori + ray->m_dir * t;
	}
	return true;
}

bool TerrainFacet::CheckInside(const Point2D& p) const
{
	Point2D p1(m_p1->x, m_p1->y);
	Point2D p2(m_p2->x, m_p2->y);
	Point2D p3(m_p3->x, m_p3->y);
	Vector2D v1 = p2 - p1;
	Vector2D v2 = p3 - p2;
	Vector2D v3 = p1 - p3;
	Vector2D vp1 = p - p1;
	Vector2D vp2 = p - p2;
	Vector2D vp3 = p - p3;
	RtLbsType c1 = Cross(v1, vp1);
	RtLbsType c2 = Cross(v2, vp2);
	RtLbsType c3 = Cross(v3, vp3);
	if ((c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0)) {//在三角形内部或在边上都判定为在三角形内部
		return true;
	}
	return false;
}

bool TerrainFacet::CheckInside(const Point3D& p) const
{
	Point2D p2d(p.x, p.y);
	Point2D p1(m_p1->x, m_p1->y);
	Point2D p2(m_p2->x, m_p2->y);
	Point2D p3(m_p3->x, m_p3->y);
	Vector2D v1 = p2 - p1;
	Vector2D v2 = p3 - p2;
	Vector2D v3 = p1 - p3;
	Vector2D vp1 = p2d - p1;
	Vector2D vp2 = p2d - p2;
	Vector2D vp3 = p2d - p3;
	RtLbsType c1 = Cross(v1, vp1);
	RtLbsType c2 = Cross(v2, vp2);
	RtLbsType c3 = Cross(v3, vp3);
	if ((c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0)) {//在三角形内部或在边上都判定为在三角形内部
		return true;
	}
	return false;
}

RtLbsType TerrainFacet::GetMinDistanceToPoint(const Point3D& p) const
{
	Vector3D ap = p - *m_p1;
	return m_normal * ap;
}

RtLbsType TerrainFacet::GetVerticleDistanceToPoint(const Point3D& p) const
{
	RtLbsType tmin = GetMinDistanceToPoint(p);
	double costheta = m_normal.z; //垂线方向和面元法向方向的夹角余弦
	if (abs(costheta) <= EPSILON) //若面元接近垂直方向，则距离为无限
		return FLT_MAX;
	return tmin / costheta;
}

RtLbsType TerrainFacet::GetFacetHeightViaPoint(const Point2D& p) const
{
	RtLbsType tarHeight = 0.0;						/** @brief	需要求解的目标高度	*/
	//构造三维点
	Point3D p3d(p.x, p.y, 0.0);
	//求解垂线距离
	RtLbsType verticalH = GetVerticleDistanceToPoint(p3d);
	//获得高度
	if (verticalH == FLT_MAX) {
		tarHeight = (m_p1->z + m_p2->z + m_p3->z) / 3;
	}
	else {
		tarHeight = 0 - verticalH;
	}
	return tarHeight;
}

Point3D TerrainFacet::GetPointOnPlane(const Point3D& p) const
{
	RtLbsType t_vert = GetVerticleDistanceToPoint(p);
	if (t_vert == FLT_MAX)
		return p; //若垂直距离为无穷，返回原坐标点
	Point3D pRe(p);
	pRe.z = p.z - t_vert;
	return pRe;
}

Point3D TerrainFacet::GetMirrorPoint(const Point3D& p) const
{
	Vector3D po = *m_p1 - p;
	RtLbsType projLen = po * m_normal;					//op 沿法向量的距离
	Point3D mirrorP = p + m_normal * projLen * 2;		//镜像点的坐标
	return mirrorP;
}

TerrainSegment* TerrainFacet::GetIntersectSegment(Ray2D* ray, TerrainSegment* prevSegment, RtLbsType* maxt, RtLbsType* height)
{
	//计算三条边是否与射线相交,需要计算线段交点与上一个节点之间的距离，若相交了除原线段之外的两个线段，则择优选择较远的线段
	TerrainSegment* reSegment = nullptr;
	RtLbsType tmax = -1.0; //交点的距离
	if (m_segment1 != prevSegment) {
		if (m_segment1->Intersect(ray, &tmax, height)) { //与射线相交的面元
			reSegment = m_segment1;
		}
	}
	if (m_segment2 != prevSegment) {
		RtLbsType t;
		if (m_segment2->Intersect(ray, &t, height)) {
			if (t > tmax) {
				tmax = t;
				reSegment = m_segment2;
			}
		}
	}
	if (m_segment3 != prevSegment) {
		RtLbsType t;
		if (m_segment3->Intersect(ray, &t, height)) {
			if (t > tmax) {
				tmax = t;
				reSegment = m_segment3;
			}
		}
	}
	*maxt = tmax;
	if (reSegment == nullptr)
		LOG_WARNING << "TerrainFacet: error limit." << ENDL;
	return reSegment;
}

bool TerrainFacet::HasEdgeSegmentIntersect(Ray2D* ray2d, RtLbsType& out_t) const
{
	out_t = FLT_MAX;
	RtLbsType cur_t = FLT_MAX;
	if (m_segment1 != nullptr && m_segment1->HasIntersect(ray2d, cur_t)) {
		if (cur_t < out_t)
			out_t = cur_t;
	}
	if (m_segment2 != nullptr && m_segment2->HasIntersect(ray2d, cur_t)) {
		if (cur_t < out_t)
			out_t = cur_t;
	}
	if (m_segment3 != nullptr && m_segment3->HasIntersect(ray2d, cur_t)) {
		if (cur_t < out_t)
			out_t = cur_t;
	}
	if (out_t < FLT_MAX)
		return true;
	return false;
}

void TerrainFacet::Update()
{
	m_bbox.InvalidBBox();					//包围盒使能无效
	m_bbox.Union(*m_p1);
	m_bbox.Union(*m_p2);
	m_bbox.Union(*m_p3);
	//计算法向量，默认p1,p2,p3为逆时针方向
	m_normal = Cross((*m_p2 - *m_p1), (*m_p3 - *m_p1)).Normalize();
}
