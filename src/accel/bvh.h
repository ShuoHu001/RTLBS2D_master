#ifndef RTLBS_BVH
#define RTLBS_BVH

#include "accelerator.h"

class BVHNode {

public:
	std::vector<Segment2D*> m_segments;
	BBox2D m_bbox;
	BVHNode* left;
	BVHNode* right;
	
public:
	BVHNode(std::vector<Segment2D*> segments);
	~BVHNode();
	bool IsLeaf() const;
};


class BVH : public Accelerator {

public:
	BVHNode* m_root;
	static const int leafSize = 20; //子节点分裂阈值
public:
	BVH();
	~BVH();
	BVHNode* Construct(std::vector<Segment2D*>& segemnts);
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	void Build() override;
	ACCEL_TYPE GetAccelType() const override;

private:
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitByAxisAlignment(std::vector<Segment2D*>& segments);//基于轴对称进行分割
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitBySAH(std::vector<Segment2D*>& segments);//基于启发式表面积进行分割
private:
	void deleteNode();//删除树结构-迭代法
};


#endif
