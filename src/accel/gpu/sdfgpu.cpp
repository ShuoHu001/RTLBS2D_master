#include "sdfgpu.h"

HOST_DEVICE_FUNC SDFNodeGPU::SDFNodeGPU()
	: m_segmentIds(nullptr)
	, m_segmentCount(0)
	, m_value(FLT_MAX)
	, id(-1)
	, x(-1)
	, y(-1)
{
}

HOST_DEVICE_FUNC  SDFNodeGPU::SDFNodeGPU(int64_t* segmentIds, size_t segmentCount, Point2D cornerPoint, RtLbsType value, int64_t _id, int64_t _x, int64_t _y)
	: m_segmentIds(segmentIds)
	, m_segmentCount(segmentCount)
	, m_cornerPoint(cornerPoint)
	, m_value(FLT_MAX)
	, id(_id)
	, x(_x)
	, y(_y)
{
}

HOST_DEVICE_FUNC  SDFNodeGPU::~SDFNodeGPU()
{
}

HOST_DEVICE_FUNC SDFNodeGPU& SDFNodeGPU::operator=(const SDFNodeGPU& node)
{
	m_segmentIds = node.m_segmentIds;
	m_segmentCount = node.m_segmentCount;
	m_cornerPoint = node.m_cornerPoint;
	m_value = node.m_value;
	id = node.id;
	x = node.x;
	y = node.y;
	return *this;
}


HOST_DEVICE_FUNC SignedDistanceFieldGPU::SignedDistanceFieldGPU()
	: m_voxels(nullptr)
	, m_voxelCount(0)
	, old_voxels(nullptr)
{
	for (unsigned i = 0; i < 2; ++i) {
		m_voxelNum[i] = 0;
		m_voxelExtent[i] = 0.0;
		m_voxelInvExtent[i] = 0.0;
	}
}

HOST_DEVICE_FUNC  SignedDistanceFieldGPU::SignedDistanceFieldGPU(BBox2DGPU bbox, SDFNodeGPU* voxels, size_t voxelNum[2], RtLbsType voxelExtent[2], RtLbsType voxelInvExtent[2], Point2D conorPoint, size_t voxelCount)
	: m_bbox(bbox)
	, m_voxels(voxels)
	, m_cornerPoint(conorPoint)
	, m_voxelCount(voxelCount)
	, old_voxels(nullptr)
{
	for (unsigned i = 0; i < 2; ++i) {
		m_voxelNum[i] = voxelNum[i];
		m_voxelExtent[i] = voxelExtent[i];
		m_voxelInvExtent[i] = voxelInvExtent[i];
	}
}

HOST_DEVICE_FUNC  SignedDistanceFieldGPU::~SignedDistanceFieldGPU()
{
}

HOST_DEVICE_FUNC SignedDistanceFieldGPU& SignedDistanceFieldGPU::operator=(const SignedDistanceFieldGPU& sdf)
{
	m_bbox = sdf.m_bbox;
	m_voxels = sdf.m_voxels;
	m_cornerPoint = sdf.m_cornerPoint;
	m_voxelCount = sdf.m_voxelCount;
	for (unsigned i = 0; i < 2; ++i) {
		m_voxelNum[i] = sdf.m_voxelNum[i];
		m_voxelExtent[i] = sdf.m_voxelExtent[i];
		m_voxelInvExtent[i] = sdf.m_voxelInvExtent[i];
	}
	return *this;
}


HOST_DEVICE_FUNC void SignedDistanceFieldGPU::GetIntersect(const Ray2DGPU& ray, Intersection2DGPU* intersect, Segment2DGPU* segmentsGPU)
{
	if (m_voxelCount == 0) {
		intersect->m_isValid = false;
		return;
	}
		
	RtLbsType maxt;
	RtLbsType cur_t = Intersect_BBox2D(ray, m_bbox, &maxt);//修改版本，射线与环境改为必然相交，若射线与环境面元不相交，则射线与环境包围盒必然相交
	//if (cur_t < 0.0) {
	//	intersect->m_isValid = false;
	//	printf("bug-finder:%f\n", cur_t);
	//	return;
	//}
	//默认以网格边界框为交点对intersect进行赋值
	intersect->m_intersect = ray(cur_t);
	intersect->m_isValid = true;
	intersect->m_ft = cur_t;
	intersect->m_type = NODE_LOS;
	intersect->m_ray = ray;
	if (cur_t == maxt)
		cur_t = 0;//射线在框内
	intersect->m_ft = thrust::min(intersect->m_ft, maxt);

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
	size_t voxelId = _offset(curGrid[0], curGrid[1]);
	Point2D curGridCorner = m_voxels[voxelId].m_cornerPoint;
	for (unsigned i = 0; i < 2; i++) {
		// get the next t
		RtLbsType target = curGridCorner[i] + ((dir[i] + 1) >> 1) * m_voxelExtent[i];
		next[i] = (target - ray.m_Ori[i]) / ray.m_Dir[i];
		if (delta[i] == FLT_MAX)
			next[i] = FLT_MAX;
	}
	//遍历网格
	RtLbsType t_total = cur_t;
	const unsigned array[] = { 0, 0, 1, 1 };//[0],[3]不可能相交
	RtLbsType jumpThreshold = 10 * m_voxelExtent[0];
	RtLbsType tracebackThrershold = 5 * m_voxelExtent[0];
	while (cur_t < intersect->m_ft) {
		
		//得到体素ID curGrid
		voxelId = _offset(curGrid[0], curGrid[1]);
		RtLbsType distanceToObstacle = m_voxels[voxelId].m_value;
		if (distanceToObstacle != 0 && distanceToObstacle > jumpThreshold) {
			distanceToObstacle -= tracebackThrershold;
			cur_t += distanceToObstacle; //先更新cur_t的值
			//跳转至体素网格,附带更新迭代部分内容
			int64_t nextGrid[2];
			for (unsigned i = 0; i < 2; ++i) {
				nextGrid[i] = _point2VoxelId(ray(cur_t), i);
				next[i] += delta[i] * abs(nextGrid[i] - curGrid[i]);
				if (nextGrid[i] < 0 || (unsigned)nextGrid[i] >= m_voxelNum[i]){
					return;
				}
				curGrid[i] = nextGrid[i];
			}
		}
		voxelId = _offset(curGrid[0], curGrid[1]);
		if (_getIntersect(voxelId, ray, intersect, segmentsGPU)) {
			return;
		}
		//if (distanceToObstacle <= next[nextAxis]) {
		//	if (_getIntersect(curGrid, ray, intersect))
		//		return true;
		//}


		//计算下一个t值
		unsigned nextAxis = (next[0] <= next[1]) + ((unsigned)(next[1] <= next[0])) * 2;
		nextAxis = array[nextAxis];

		// get to the next voxel
		curGrid[nextAxis] += dir[nextAxis];

		if (curGrid[nextAxis] < 0 || (unsigned)curGrid[nextAxis] >= m_voxelNum[nextAxis]) {
			return;
		}

		//更新下一个单元
		cur_t = next[nextAxis];
		next[nextAxis] += delta[nextAxis];
	}
	return;
}

HOST_DEVICE_FUNC bool SignedDistanceFieldGPU::IsRayCaptureByWedgePoint(const Ray2DGPU& ray, Point2D& p, SignedDistanceFieldGPU* sdf, Segment2DGPU* segments) const
{
	//计算广义源位置
	RtLbsType t = ray.m_tMax - ray.m_tMin;//与广义源的距离
	Point2D vSource = ray.m_Ori + ray.m_Dir * -t;
	Vector2D op = (p - vSource).Normalize();
	double op_costheta = op * ray.m_Dir;
	if (op_costheta < ray.m_costheta)//op 张角大于射线张角，未被射线管捕捉
		return false;
	Ray2DGPU newRay(ray);
	newRay.m_Ori = vSource;
	newRay.m_Dir = op;
	//下面分为广义源和非广义源进行讨论：广义源无需修正路径节点，非广义源需要修正路径节点

	if (ray.m_nodeType == NODE_ROOT || ray.m_nodeType == NODE_DIFF) {//广义源情形判定
		Intersection2DGPU inter1;
		sdf->GetIntersect(newRay, &inter1, segments);
		if (!inter1.m_isValid)//不相交
			return false;
		if (inter1.m_intersect != p)
			return false;
		//*intersect = inter1;
		return true;
	}

	//非广义源情况
	Intersection2DGPU inter1;
	const Segment2DGPU& segment = segments[ray.m_primitiveId];
	if (!segment.GetIntersect(newRay, &inter1))
		return false;
	Vector2D ip = (p - inter1.m_intersect).Normalize();
	if (op * ip < 0)//p 点在面元后方
		return false;
	newRay.m_Ori = inter1.m_intersect;
	Intersection2DGPU inter2;
	sdf->GetIntersect(newRay, &inter2, segments);
	if (!inter2.m_isValid)
		return false;
	if (inter2.m_intersect != p)
		return false;
	//*intersect = inter2;
	return true;
	//这里由于GPU的性能限制，不进行重复绕射判定
}

void SignedDistanceFieldGPU::AllocateOnDevice()
{
	cudaError_t cudaStatus;
	if (device_sdfGPU == nullptr) {//这里采用了指针技巧，改变了指向本地内存的指针，使其指向GPU内存，并在结束分配后还原
		old_segmentIds = new int64_t * [m_voxelCount];
		for (size_t i = 0; i < m_voxelCount; ++i) { //修改数组SDFNodeGPU的指针指向
			if (m_voxels[i].m_segmentCount != 0) {
				int64_t* d_segmentIds = nullptr;
				cudaStatus = cudaMalloc(&d_segmentIds, m_voxels[i].m_segmentCount * sizeof(int64_t));
				if (cudaStatus != cudaSuccess) {
					LOG_ERROR << "GPU memory allocate failed!" << CRASH;
				}
				cudaStatus = cudaMemcpy(d_segmentIds, m_voxels[i].m_segmentIds, m_voxels[i].m_segmentCount * sizeof(int64_t), cudaMemcpyHostToDevice);
				if (cudaStatus != cudaSuccess) {
					LOG_ERROR << "GPU memory copy failed!" << CRASH;
				}
				// Replace the host pointer with the device pointer.
				old_segmentIds[i] = m_voxels[i].m_segmentIds;
				m_voxels[i].m_segmentIds = d_segmentIds;
			}
		}

		SDFNodeGPU* d_voxels = nullptr;
		cudaStatus = cudaMalloc(&d_voxels, m_voxelCount * sizeof(SDFNodeGPU));
		if (cudaStatus != cudaSuccess) {
			LOG_ERROR << "GPU memory allocate failed!" << CRASH;
		}
		cudaStatus = cudaMemcpy(d_voxels, m_voxels, m_voxelCount * sizeof(SDFNodeGPU), cudaMemcpyHostToDevice);
		if (cudaStatus != cudaSuccess) {
			LOG_ERROR << "GPU memory copy failed!" << CRASH;
		}

		old_voxels = m_voxels;
		m_voxels = d_voxels;

		cudaStatus = cudaMalloc(&device_sdfGPU, sizeof(SignedDistanceFieldGPU));
		if (cudaStatus != cudaSuccess) {
			LOG_ERROR << "GPU memory allocate failed!" << CRASH;
		}
		cudaStatus = cudaMemcpy(device_sdfGPU, this, sizeof(SignedDistanceFieldGPU), cudaMemcpyHostToDevice);
		if (cudaStatus != cudaSuccess) {
			LOG_ERROR << "GPU memory copy failed!" << CRASH;
		}
	}
}

void SignedDistanceFieldGPU::ReleaseOnDevice()
{
	if (device_sdfGPU != nullptr) {
		SDFNodeGPU* d_voxels = m_voxels;
		m_voxels = old_voxels;//恢复主机地址数据
		cudaError_t cudaStatus;
		for (size_t i = 0; i < m_voxelCount; ++i) {
			if (m_voxels[i].m_segmentCount != 0) {
				cudaStatus = cudaFree(m_voxels[i].m_segmentIds);
				if (cudaStatus != cudaSuccess) {
					LOG_ERROR << "GPU memory free failed!" << CRASH;
				}
			}
		}
		m_voxels = d_voxels;
		cudaStatus = cudaFree(m_voxels);
		if (cudaStatus != cudaSuccess) {
			LOG_ERROR << "GPU memory free failed!" << CRASH;
		}
		cudaStatus = cudaFree(device_sdfGPU);
		if (cudaStatus != cudaSuccess) {
			LOG_ERROR << "GPU memory free failed!" << CRASH;
		}
	}
	//释放GPU内存后还原指针还原指针
	m_voxels = old_voxels;
	for (size_t i = 0; i < m_voxelCount; ++i) {
		m_voxels[i].m_segmentIds = old_segmentIds[i];  // Restore the original host pointer.
	}
	delete[] old_segmentIds;
}

SignedDistanceFieldGPU* SignedDistanceFieldGPU::GetDevicePointer() const
{
	return device_sdfGPU;
}

HOST_DEVICE_FUNC bool SignedDistanceFieldGPU::_getIntersect(int64_t id, const Ray2DGPU& ray, Intersection2DGPU* intersect, Segment2DGPU* segmentsGPU) const
{
	if (m_voxels[id].m_segmentCount == 0)
		return false;
	//if (intersect)
	//	intersect->m_ft = FLT_MAX;//重置intersect的值
	bool hasIntersect = false;
	Intersection2DGPU curIntersect;//当前的交点
	for (int i = 0; i < m_voxels[id].m_segmentCount; ++i) {
		int64_t& segmentId = m_voxels[id].m_segmentIds[i];
		Segment2DGPU& segment = segmentsGPU[segmentId];
		if (segment.GetIntersect(ray, &curIntersect)) {
			if (!intersect) //若intersect为nullptr直接返回true
				return true;
			if (curIntersect.m_ft < intersect->m_ft)
				*intersect = curIntersect;
			hasIntersect = true;
		}
	}
	return hasIntersect;
}

HOST_DEVICE_FUNC int64_t SignedDistanceFieldGPU::_point2VoxelId(const Point2D& p, unsigned axis) const
{
	return thrust::min((int64_t)m_voxelNum[axis] - 1, (int64_t)((p[axis] - m_bbox.m_Min[axis]) * m_voxelInvExtent[axis]));
}

HOST_DEVICE_FUNC int64_t SignedDistanceFieldGPU::_offset(int64_t x, int64_t y) const
{
	return y * m_voxelNum[0] + x;
}

