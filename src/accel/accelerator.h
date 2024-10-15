#ifndef RTLBS_ACCELERATOR
#define RTLBS_ACCELERATOR

//ͷ�ļ�����
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
	virtual bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const = 0;				//���麯��������̳б���ʵ��
	virtual void Build() = 0;																		//�������ٽṹ�������������ʵ��
	virtual ACCEL_TYPE GetAccelType() const = 0;													//��ü�����ṹ�����ͣ��������ʵ��
	std::vector<Segment2D*> GetSegments();
	BBox2D GetBBox();
};


#endif
