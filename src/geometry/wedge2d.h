#ifndef RTLBS_WEDGE2D
#define RTLBS_WEDGE2D

#include "rtlbs.h"
#include "utility/define.h"
#include "point2d.h"
#include "vector3d.h"
#include "gpu/wedge2dgpu.h"

class Segment2D;

class Wedge2D {
public:
	int m_globalId;				/** @brief	棱角编号,全局ID	*/
	int m_privateId;			/** @brief	棱角编号，内部ID	*/
	Segment2D* m_face1;			/** @brief	面元1	*/
	Segment2D* m_face2;			/** @brief	面元2	*/
	Point2D m_point;			/** @brief	棱劈点	*/
	double m_theta;				/** @brief	外角（弧度）	*/
	Vector2D m_dir1;			/** @brief	沿着face1方向的向量	*/
	Vector2D m_dir2;			/** @brief	沿着face2方向的向量	*/

public:
	Wedge2D();
	Wedge2D(int privateId, Segment2D* face1, Segment2D* face2, Point2D p);
	Wedge2D(int privateId, Segment2D* face1, Segment2D* face2, Point2D p, double theta, Vector2D vector1, Vector2D vector2);
	~Wedge2D() {};

	bool operator == (const Wedge2D& other) const;
	bool operator != (const Wedge2D& other) const;
	Wedge2D& operator = (const Wedge2D& wedge);
	bool IsContainsPoint(const Point2D& point); //判断楔形角是否包含某个坐标点
	Wedge2DGPU Convert2GPU();
	RtLbsType GetNValue() const;						//计算棱劈角的N值
	void CalDiffractionParameters(const Vector3D& inDir, const Vector3D& diffDir, const Point3D& interPoint, RtLbsType& incidentTheta, RtLbsType& diffractionTheta) const;

};

#endif
