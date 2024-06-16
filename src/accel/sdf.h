#ifndef RTLBS_SDF
#define RTLBS_SDF

#include "rtlbs.h"
#include "utility/define.h"
#include "accelerator.h"
#include "gridid.h"
#include "accel/gpu/sdfgpu.h"



class SDFNode {
public:
	std::vector<Segment2D*> m_segemnts; /** @brief	�ڵ��а�������Ԫ	*/
	Point2D m_cornerPoint; /** @brief	�������½ǽǵ�����	*/
	int64_t x, y;           /** @brief	�ڵ�Ķ�ά����ֵ�����ڼ�����볡	*/
	int64_t id;				/** @brief	�ڵ��һά����ֵ	*/
	RtLbsType m_value; /** @brief	���볡ֵ, ��ʼֵΪFLTMAX	*/
public:
	SDFNode();
	~SDFNode();
	bool operator<(const SDFNode& node) const; /** @brief	���볡����Ƚϣ��������ȶ�������С��	*/

	SDFNodeGPU Convert2GPU();
};



class SignedDistanceField :public Accelerator {
public:
	std::vector<SDFNode> m_voxels;		/** @brief	��������	*/
	int64_t m_voxelNum[2];						/** @brief	��������	*/
	RtLbsType m_voxelExtent[2];				/** @brief	����߳�	*/
	RtLbsType m_voxelInvExtent[2];			/** @brief	����߳��ĵ���	*/
	Point2D m_cornerPoint;					/** @brief	�������½ǵ�����	*/
	size_t m_voxelCount;						/** @brief	��������	*/
public:
	SignedDistanceField();
	~SignedDistanceField();
	void Build() override;
	bool GetIntersect(const Ray2D& ray, Intersection2D* intersect) const override;
	SignedDistanceFieldGPU Convert2GPU();// �����볡ת��ΪGPU�ϵľ��볡
	ACCEL_TYPE GetAccelType() const override;

private:
	std::vector<int64_t> m_source; /** @brief	���볡�еĵ�Դ(��������Ԫ���������)	*/

private:
	void _init();
	void _constructBasicGrid(std::vector<Segment2D*>& segments);		/** @brief	������������	*/
	void _constructSDFGrid();									/** @brief	�������볡	*/
	std::vector<int64_t> _getGridCoordAlongSegment(Segment2D& segment); //����segment���һϵ�е�������
	bool _getIntersect(int64_t id, const Ray2D& ray, Intersection2D* intersect) const;//������voxel��
	int64_t _point2VoxelId(const Point2D& p, unsigned axis) const;
	int64_t _offset(int64_t x, int64_t y) const; //�����ά������һά����֮���ӳ���ϵ
};

#endif
