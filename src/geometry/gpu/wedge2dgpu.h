#ifndef RTLBS_WEDGE2DGPU
#define RTLBS_WEDGE2DGPU


#include "geometry/point2d.h"
#include "segment2dgpu.h"

class Wedge2DGPU {
public:
	int m_wedge_id; /** @brief	棱角编号	*/
	Segment2DGPU m_face1;
	Segment2DGPU m_face2;
	Point2D m_edge;  
	double m_fExternalAngle; /** @brief	外角（弧度）	*/
	Vector2D m_vector1;  /** @brief	沿着face1方向的向量	*/
	Vector2D m_vector2;  /** @brief	沿着face2方向的向量	*/

public:
	HOST_DEVICE_FUNC Wedge2DGPU(): m_fExternalAngle(0), m_wedge_id(0){}
	HOST_DEVICE_FUNC Wedge2DGPU(const Wedge2DGPU& wedge);
	HOST_DEVICE_FUNC ~Wedge2DGPU() {};

	HOST_DEVICE_FUNC bool operator == (const Wedge2DGPU& other) const;
	HOST_DEVICE_FUNC bool operator != (const Wedge2DGPU& other) const;
	HOST_DEVICE_FUNC Wedge2DGPU& operator = (const Wedge2DGPU& wedge);
};

#endif
