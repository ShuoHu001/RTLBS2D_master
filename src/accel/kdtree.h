#ifndef RTLBS_KDTREE
#define RTLBS_KDTREE

#include "rtlbs.h"
#include "utility/define.h"
#include "accelerator.h"

class KDTreeNode {
public:
	std::vector<Segment2D*> m_segments;	//�ڵ��б����ͼԪ
	int m_axis;						//���ڷָ����
	BBox2D m_bbox;					//��Χ��
	KDTreeNode* left;
	KDTreeNode* right;
	
public:
	KDTreeNode();
	~KDTreeNode();
	bool IsLeaf() const;
};

class KDTree :public Accelerator {
public:
	KDTreeNode* m_root;
	static const int leafSize = 10; //�ӽڵ������ֵ 1-20Ϊ��ֵ��Χ��1��Ӧ�ڷֲ���Ϊ���У�20��Ӧ�ڷֲ���Ϊ��ɢ
public:
	KDTree();
	~KDTree();
	KDTreeNode* Construct(std::vector<Segment2D*>& segments);
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	void Build() override;
	ACCEL_TYPE GetAccelType() const override;
private:
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitByMiddlePoint(std::vector<Segment2D*>& segments);//�����е���л���
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitBySAH(std::vector<Segment2D*>& segments, int& splitaxis);//��������ʽ��������зָ�
	std::pair<RtLbsType*, RtLbsType*> computeVariances(const std::vector<Segment2D*>& segments); //����ͼԪ�����᷽��
	void deleteNode(); //�ݹ�ɾ���ڵ�
};


#endif
