#ifndef RTLBS_ACCELERATOR
#define RTLBS_ACCELERATOR

//头文件包含
#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/segment2d.h"
#include "geometry/Intersection2D.h"
#include "gpu/sdfgpu.h"

class Accelerator {
	protected:
	std::vector<Segment2D*>* m_segments;
	BBox2D m_bbox;

public:
	Accelerator();
	void SetPrimitives(std::vector<Segment2D*>* segments);
	virtual ~Accelerator() { }
	virtual bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const = 0;				//纯虚函数，子类继承必须实现
	virtual void Build() = 0;																		//构建加速结构函数，子类必须实现
	virtual ACCEL_TYPE GetAccelType() const = 0;													//获得加速体结构的类型，子类必须实现
	std::vector<Segment2D*> GetSegments();
	BBox2D GetBBox();
};


#endif
