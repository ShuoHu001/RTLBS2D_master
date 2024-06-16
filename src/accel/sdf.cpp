#include "sdf.h"
#include <fstream>
#include <queue>

SDFNode::SDFNode()
	: x(-1)
	, y(-1)
	, id(-1)
	, m_value(FLT_MAX)
{
}

SDFNode::~SDFNode()
{
}

bool SDFNode::operator<(const SDFNode& node) const
{
	return m_value > node.m_value;
}

SDFNodeGPU SDFNode::Convert2GPU()
{
	SDFNodeGPU nodeGPU;
	nodeGPU.id = id;
	nodeGPU.x = x;
	nodeGPU.y = y;
	nodeGPU.m_segmentCount = m_segemnts.size();
	if (!m_segemnts.empty()) {
		nodeGPU.m_segmentIds = new int64_t[nodeGPU.m_segmentCount];
		for (size_t i = 0; i < nodeGPU.m_segmentCount; ++i) {
			nodeGPU.m_segmentIds[i] = m_segemnts[i]->m_id;
		}
	}
	nodeGPU.m_cornerPoint = m_cornerPoint;
	nodeGPU.m_value = m_value;
	return nodeGPU;
}

SignedDistanceField::SignedDistanceField()
{
	_init();
}

SignedDistanceField::~SignedDistanceField()
{
}

void SignedDistanceField::Build()
{
	_constructBasicGrid(*m_segments); //����������������
	_constructSDFGrid();//�������볡
}

bool SignedDistanceField::GetIntersect(const Ray2D& ray, Intersection2D* intersect) const
{
	if (m_voxelCount == 0 || m_segments == 0)
		return false;
	RtLbsType maxt;
	RtLbsType cur_t = Intersect_BBox2D(ray, m_bbox, &maxt);
	if (cur_t < 0.0)
		return false;
	if (cur_t == maxt)
		cur_t = 0;//�����ڿ���
	if (intersect)
		intersect->m_ft = std::min(intersect->m_ft, maxt);

	int64_t curGrid[2], dir[2];
	RtLbsType delta[2], next[2];

	for (unsigned i = 0; i < 2; i++) {
		curGrid[i] = _point2VoxelId(ray(cur_t), i);
		dir[i] = (ray.m_Dir[i] > 0.0) ? 1 : -1;
		if (ray.m_Dir[i] != 0.0)
			delta[i] = abs(m_voxelExtent[i] / ray.m_Dir[i]);
		else
			delta[i] = FLT_MAX;
	}

	int64_t voxelId = _offset(curGrid[0], curGrid[1]);
	Point2D curGridCorner = m_voxels[voxelId].m_cornerPoint;
	for (unsigned i = 0; i < 2; i++) {
		// get the next t
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - ray.m_Ori[i]) / ray.m_Dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}
	//��������
	RtLbsType t_total = cur_t;
	const unsigned array[] = { 0, 0, 1, 1 };//[0],[3]�������ཻ
	RtLbsType jumpThreshold = 10 * m_voxelExtent[0];
	RtLbsType tracebackThrershold = 5 * m_voxelExtent[0];
	while (intersect && cur_t < intersect->m_ft || (intersect == 0)) {

		//�õ�����ID curGrid
		voxelId = _offset(curGrid[0], curGrid[1]);
		RtLbsType distanceToObstacle = m_voxels[voxelId].m_value;
		if (distanceToObstacle != 0 && distanceToObstacle > jumpThreshold) {
			distanceToObstacle -= tracebackThrershold;
			cur_t += distanceToObstacle; //�ȸ���cur_t��ֵ
			//��ת����������,�������µ�����������
			GridId nextGrid;
			for (unsigned i = 0; i < 2; ++i) {
				nextGrid[i] = _point2VoxelId(ray(cur_t), i);
				next[i] += delta[i] * abs(nextGrid[i] - curGrid[i]);
				if (nextGrid[i] < 0 || (unsigned)nextGrid[i] >= m_voxelNum[i])
					return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));
				curGrid[i] = nextGrid[i];
			}
		}
		voxelId = _offset(curGrid[0], curGrid[1]);
		if (_getIntersect(voxelId, ray, intersect))
			return true;
		//if (distanceToObstacle <= next[nextAxis]) {
		//	if (_getIntersect(curGrid, ray, intersect))
		//		return true;
		//}


		//������һ��tֵ
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];

		// get to the next voxel
		curGrid[nextAxis] += dir[nextAxis];

		if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis])
			return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));

		//������һ����Ԫ
		cur_t = next[nextAxis];
		next[nextAxis] += delta[nextAxis];
	}

	return (intersect && intersect->m_ft < maxt && (intersect->m_segment != 0));
}

SignedDistanceFieldGPU SignedDistanceField::Convert2GPU()
{

	SignedDistanceFieldGPU sdfGPU;
	sdfGPU.m_bbox = m_bbox.Convert2GPU();
	for (unsigned i = 0; i < 2; ++i) {
		sdfGPU.m_voxelNum[i] = m_voxelNum[i];
		sdfGPU.m_voxelExtent[i] = m_voxelExtent[i];
		sdfGPU.m_voxelInvExtent[i] = m_voxelInvExtent[i];
	}
	sdfGPU.m_cornerPoint = m_cornerPoint;
	sdfGPU.m_voxelCount = m_voxelCount;
	sdfGPU.m_voxels = new SDFNodeGPU[m_voxelCount];
	for (size_t i = 0; i < m_voxelCount; ++i) {
		sdfGPU.m_voxels[i] = m_voxels[i].Convert2GPU();
	}
	return sdfGPU;
}

ACCEL_TYPE SignedDistanceField::GetAccelType() const
{
	return ACCEL_SDF;
}




void SignedDistanceField::_init()
{
	for (unsigned i = 0; i < 2; i++) {
		m_voxelNum[i] = 0;
		m_voxelExtent[i] = 0.0;
		m_voxelInvExtent[i] = 0.0;
	}
	m_voxelCount = 0;
}

void SignedDistanceField::_constructBasicGrid(std::vector<Segment2D*>& segments)
{
	//���������С
	Vector2D delta = m_bbox.m_max - m_bbox.m_min;
	RtLbsType extent = delta.MaxComponent();
	size_t count = segments.size();
	RtLbsType gridPerDistance = static_cast<RtLbsType>(sqrt(count / extent));

	for (unsigned i = 0; i < 2; i++) {//x�ᡢy��
		m_voxelNum[i] = static_cast<int64_t>(ceil(std::min(512.0, gridPerDistance * delta[i])));
		m_voxelInvExtent[i] = m_voxelNum[i] / delta[i];
		m_voxelExtent[i] = 1.0 / m_voxelInvExtent[i];
	}

	//�����ʼ��
	m_voxelCount = m_voxelNum[0] * m_voxelNum[1];
	m_cornerPoint = m_bbox.m_min;
	m_voxels.resize(m_voxelCount);
	int64_t k = 0;
	for (int64_t i = 0; i < m_voxelNum[0]; ++i) { //�ǵ㸳ֵ
		for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
			k = j * m_voxelNum[0] + i;
			m_voxels[k].m_cornerPoint[0] = m_cornerPoint[0] + i * m_voxelExtent[0];
			m_voxels[k].m_cornerPoint[1] = m_cornerPoint[1] + j * m_voxelExtent[1];
			m_voxels[k].x = i;
			m_voxels[k].y = j;
			m_voxels[k].id = k;
		}
	}
	

	//ѭ����Ԫ���и�ֵ
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		Segment2D* segment = *it;
		std::vector<int64_t> Ids = _getGridCoordAlongSegment(*segment);
		m_source.insert(m_source.end(), Ids.begin(), Ids.end());//�����볡�ĵ�Դ��ֵ
		if (Ids.empty())
			LOG_WARNING << "empty vector" << ENDL;
		for (size_t i = 0; i < Ids.size(); ++i) {
			m_voxels[Ids[i]].m_segemnts.push_back(segment);
		}
	}


	std::ofstream outFile;
	outFile.open("UniGrid-1.txt");
	for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
		for (int64_t i = 0; i < m_voxelNum[0]; ++i) {
			outFile << static_cast<int64_t>(m_voxels[j * m_voxelNum[0] + i].m_segemnts.size()) << " ";
		}
		outFile << "\n";
	}
	outFile.close();

}

void SignedDistanceField::_constructSDFGrid()
{
	//1-��ʼ��Դֵ
	for (auto it = m_source.begin(); it != m_source.end(); ++it) {
		const size_t& id = *it;
		m_voxels[id].m_value = 0.0; /** @brief	Դ��ĳ�ֵ��ʼ��Ϊ0.0	*/
	}

	//2-ͨ�����ȶ��е������¼�����볡
	std::priority_queue<SDFNode> nodeQueue;
	for (auto it = m_source.begin(); it != m_source.end(); ++it) {
		const size_t& id = *it;
		nodeQueue.push(m_voxels[id]);
	}

	//3-FastMarchingMethod ��ѭ��
	while (!nodeQueue.empty()) {
		SDFNode current = nodeQueue.top();
		nodeQueue.pop();

		//�����Χ�Ľڵ�
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				if (dx == 0 && dy == 0) 
					continue; //��������ڵ�

				int64_t newX = current.x + dx;
				int64_t newY = current.y + dy;

				//���߽�
				if (newX<0 || newX>(m_voxelNum[0] - 1) || newY<0 || newY>(m_voxelNum[1] - 1))
					continue; //��ֹ��鳬���߽������
				//�����µľ���
				int64_t voxelId = _offset(newX, newY);
				SDFNode& nextNode = m_voxels[voxelId];
				if (nextNode.m_value == 0.0) /** @brief	��һ���ڵ�Ϊ��Դ�ڵ㣬ֱ�������������о������	*/
					continue;
				RtLbsType newDist = current.m_value + (nextNode.m_cornerPoint - current.m_cornerPoint).Length();

				if (newDist < nextNode.m_value) { //����һ���ڵ�ľ���С�ڵ�ǰ�ڵ�ľ��룬����¾��벢����һ���ڵ����뵽������
					nextNode.m_value = newDist;
					nodeQueue.push(nextNode);
				}
			}
		}
	}


	//test ���֣�����ǰ�ľ��볡д���ļ���

	std::ofstream outFile;
	outFile.open("SDF.txt");
	for (int64_t j = 0; j < m_voxelNum[1]; ++j) {
		for (int64_t i = 0; i < m_voxelNum[0]; ++i) {
			outFile << m_voxels[j* m_voxelNum[0] + i].m_value << " ";
		}
		outFile << "\n";
	}
	outFile.close();

}

std::vector<int64_t>  SignedDistanceField::_getGridCoordAlongSegment(Segment2D& segment)
{
	std::vector<int64_t>  Ids;

	////����-1 ֱ�Ӷ԰�Χ�н��и�ֵ
	//for (int i = sId.x; i <= eId.x; ++i) {
	//	for (int j = sId.y; j <= eId.y; ++j) {
	//		Ids.push_back(GridId(i, j));
	//	}
	//}

	//����2-������������㷨���и�ֵ(��ȷ)
	int64_t curGrid[2], endGrid[2], dir[2];//��ǰ�������š���ֹ�������š��н�����
	RtLbsType delta[2], next[2];
	RtLbsType maxt = segment.m_length;		/** @brief	����׷�ٵ�������	*/
	RtLbsType curt = 0;						/** @brief	��ǰ�ľ���	*/
	Vector2D& s_dir = segment.m_dir;
	Point2D& sp = segment.m_ps;
	Point2D& ep = segment.m_pe;

	//���㵥λ����ֵ
	for (unsigned i = 0; i < 2; i++) {
		curGrid[i] = _point2VoxelId(sp, i);
		endGrid[i] = _point2VoxelId(ep, i);
		dir[i] = (s_dir[i] > 0.0) ? 1 : -1;
		if (s_dir[i] != 0.0)
			delta[i] = abs(m_voxelExtent[i] / s_dir[i]);
		else
			delta[i] = FLT_MAX;
	}

	//����Ŀ��ֵ
	int64_t voxelId = _offset(curGrid[0], curGrid[1]);
	Point2D& curGridCorner = m_voxels[voxelId].m_cornerPoint;
	for (unsigned i = 0; i < 2; i++) {
		// get the next t
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - sp[i]) / s_dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}

	//��������������ر��
	//��������
	const unsigned array[] = { 0, 0, 1, 1 };//[0],[3]�������ཻ
	while (curt < maxt) {
		voxelId = _offset(curGrid[0], curGrid[1]);
		Ids.push_back(voxelId);//����ǰ̽�⵽�������ż�¼��vector��
		//������һ��tֵ
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];

		if (curt < 0) {
			LOG_ERROR << "_getGridCoordAlongSegment:" << "curt < 0" << CRASH;
		}

		// get to the next voxel
		curGrid[nextAxis] += dir[nextAxis];
		curt = next[nextAxis];
		next[nextAxis] += delta[nextAxis];
	}
	return Ids;
}


bool SignedDistanceField::_getIntersect(int64_t id, const Ray2D& ray, Intersection2D* intersect) const
{
	if (m_voxels[id].m_segemnts.empty())
		return false;
	if (intersect)
		intersect->m_ft = FLT_MAX;//����intersect��ֵ
	bool hasIntersect = false;
	Intersection2D curIntersect;//��ǰ�Ľ���
	for (auto it = m_voxels[id].m_segemnts.begin(); it != m_voxels[id].m_segemnts.end(); ++it) {
		Segment2D* segment = *it;
		if (segment->GetIntersectNoBBox(ray, &curIntersect)) {
			if (!intersect) //��intersectΪnullptrֱ�ӷ���true
				return true;
			if (curIntersect.m_ft < intersect->m_ft)
				*intersect = curIntersect;
			hasIntersect = true;
		}
	}
	return hasIntersect;
}

int64_t SignedDistanceField::_point2VoxelId(const Point2D& p, unsigned axis) const
{
	return std::min(static_cast<int64_t>(m_voxelNum[axis] - 1), static_cast<int64_t>((p[axis] - m_bbox.m_min[axis]) * m_voxelInvExtent[axis]));
}

int64_t SignedDistanceField::_offset(int64_t x, int64_t y) const
{
	return y * m_voxelNum[0] + x;
}
