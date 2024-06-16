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
	int m_globalId;				/** @brief	��Ǳ��,ȫ��ID	*/
	int m_privateId;			/** @brief	��Ǳ�ţ��ڲ�ID	*/
	Segment2D* m_face1;			/** @brief	��Ԫ1	*/
	Segment2D* m_face2;			/** @brief	��Ԫ2	*/
	Point2D m_point;			/** @brief	������	*/
	double m_theta;				/** @brief	��ǣ����ȣ�	*/
	Vector2D m_dir1;			/** @brief	����face1���������	*/
	Vector2D m_dir2;			/** @brief	����face2���������	*/

public:
	Wedge2D();
	Wedge2D(int privateId, Segment2D* face1, Segment2D* face2, Point2D p);
	Wedge2D(int privateId, Segment2D* face1, Segment2D* face2, Point2D p, double theta, Vector2D vector1, Vector2D vector2);
	~Wedge2D() {};

	bool operator == (const Wedge2D& other) const;
	bool operator != (const Wedge2D& other) const;
	Wedge2D& operator = (const Wedge2D& wedge);
	bool IsContainsPoint(const Point2D& point); //�ж�Ш�ν��Ƿ����ĳ�������
	Wedge2DGPU Convert2GPU();
	RtLbsType GetNValue() const;						//���������ǵ�Nֵ
	void CalDiffractionParameters(const Vector3D& inDir, const Vector3D& diffDir, const Point3D& interPoint, RtLbsType& incidentTheta, RtLbsType& diffractionTheta) const;

};

#endif
