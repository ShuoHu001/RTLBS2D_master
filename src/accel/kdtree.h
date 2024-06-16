#ifndef RTLBS_KDTREE
#define RTLBS_KDTREE

#include "rtlbs.h"
#include "utility/define.h"
#include "accelerator.h"

class KDTreeNode {
public:
	std::vector<Segment2D*> m_segments;	//节点中保存的图元
	int m_axis;						//用于分割的轴
	BBox2D m_bbox;					//包围盒
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
	static const int leafSize = 10; //子节点分裂阈值 1-20为阈值范围，1对应于分布较为集中，20对应于分布较为分散
public:
	KDTree();
	~KDTree();
	KDTreeNode* Construct(std::vector<Segment2D*>& segments);
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	void Build() override;
	ACCEL_TYPE GetAccelType() const override;
private:
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitByMiddlePoint(std::vector<Segment2D*>& segments);//基于中点进行划分
	std::pair<std::vector<Segment2D*>, std::vector<Segment2D*>> splitBySAH(std::vector<Segment2D*>& segments, int& splitaxis);//基于启发式表面积进行分割
	std::pair<RtLbsType*, RtLbsType*> computeVariances(const std::vector<Segment2D*>& segments); //给定图元计算轴方差
	void deleteNode(); //递归删除节点
};


#endif
