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
	static const int leafSize = 20; //�ӽڵ������ֵ
public:
	BVH();
	~BVH();
	BVHNode* Construct(std::vector<Segment2D*>& segemnts);
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	void Build() override;
	ACCEL_TYPE GetAccelType() const override;

private:
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitByAxisAlignment(std::vector<Segment2D*>& segments);//������Գƽ��зָ�
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitBySAH(std::vector<Segment2D*>& segments);//��������ʽ��������зָ�
private:
	void deleteNode();//ɾ�����ṹ-������
};


#endif
