#include "transmissiongpu.h"

HOST_DEVICE_FUNC bool GenerateTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay) {
	if (!inter.m_propagationProperty.m_hasTransmission)						//若无透射属性，则不产生透射射线
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	PATHNODETYPE tranType = NODE_TRANIN;
	if (incident_ray.m_fRefractiveIndex == segment.m_refractN) {
		tranType = NODE_TRANOUT;
	}
	if (segment.m_refractN == segment.m_refractNOut) {//介质为“空气”介质，不改变传播方向
		*newRay = incident_ray;
		newRay->m_isValid = true;
		newRay->m_Ori = inter.m_intersect;
		newRay->m_fMin = incident_ray.m_fMin;
		newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
		newRay->m_primitiveId = inter.m_segmentId;
		newRay->m_nodeType = tranType;
		newRay->m_prevInterId = prevInterId;
		return true;
	}
	RtLbsType n1, n2;//n1:前一种介质的折射率，n2:后一种介质的折射率
	n1 = incident_ray.m_fRefractiveIndex;
	n2 = segment.m_refractN;                                           //默认射线从外部→内部传播，采用介质折射率
	Vector2D I = incident_ray.m_Dir;
	Vector2D N = segment.m_normal;
	if (n1 == n2) {//射线上的折射率与面元内部折射率相同，射线方向：内部→外部
		N = -N;//内部情况下，扭转面元法向
		n2 = segment.m_refractNOut;
	}
	double dotIN = I * N;
	double critical_angle = asin(n2 / n1);
	double incident_angle = acos(-dotIN);
	if (incident_angle > critical_angle)//大于临界角，不发生透射
		return false;
	RtLbsType ratio = n1 / n2;


	Vector2D I_prep = I - static_cast<RtLbsType>(dotIN) * N; //入射方向沿法向量垂直方向的投影向量
	Vector2D T_prep = ratio * I_prep; //透射方向沿法向量垂直方向的投影分量
	double cos_theta_T = sqrt(1.0 - T_prep.SquaredLength());
	Vector2D T_N = static_cast<RtLbsType>(-1.0 * cos_theta_T) * N;//透射法向沿法向量方向上的分量


	newRay->m_Dir = T_prep + T_N;
	if (isnan(newRay->m_Dir.x))
		printf("计算到临界角，请检查错误");
	*newRay = incident_ray;
	newRay->m_isValid = true;
	newRay->m_limTotl = incident_ray.m_limTotl - 1;
	newRay->m_limTran = incident_ray.m_limTran - 1;
	newRay->m_Ori = inter.m_intersect;
	newRay->m_fRefractiveIndex = n2; /** @brief	进入到"出射介质体中"	*/
	newRay->m_fMin = incident_ray.m_fMin;
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	newRay->m_bsplit = false; //透射射线不具备分裂属性
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;
	return true;
}

HOST_DEVICE_FUNC bool GenerateEmpiricalTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay)
{
	if (!inter.m_propagationProperty.m_hasEmpiricalTransmission)				//若无经验透射属性，则不产生经验透射属性
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	PATHNODETYPE tranType = NODE_ETRANIN;
	if (incident_ray.m_fRefractiveIndex == segment.m_refractN) {
		tranType = NODE_ETRANOUT;
	}
	*newRay = incident_ray;
	newRay->m_isValid = true;
	newRay->m_Ori = inter.m_intersect;
	newRay->m_fMin = incident_ray.m_fMin;
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;
	return true;
}

HOST_DEVICE_FUNC bool GenerateTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay, TreeNodeGPU* node, int layer)
{
	if (!inter.m_propagationProperty.m_hasTransmission)						//若无透射属性，则不产生透射射线
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	if (incident_ray.m_nodeType == NODE_DIFF) {				//绕射后不进行透射
		return false;
	}
	PATHNODETYPE tranType = NODE_TRANIN;
	if (incident_ray.m_fRefractiveIndex == segment.m_refractN) {
		tranType = NODE_TRANOUT;
	}
	if (segment.m_refractN == segment.m_refractNOut) {//介质为“空气”介质，不改变传播方向
		*newRay = incident_ray;
		newRay->m_isValid = true;
		newRay->m_Ori = inter.m_intersect;
		newRay->m_fMin = incident_ray.m_fMin;
		newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
		newRay->m_primitiveId = inter.m_segmentId;
		newRay->m_nodeType = tranType;
		newRay->m_prevInterId = prevInterId;
		return true;
	}
	RtLbsType n1, n2;//n1:前一种介质的折射率，n2:后一种介质的折射率
	n1 = incident_ray.m_fRefractiveIndex;
	n2 = segment.m_refractN;                                           //默认射线从外部→内部传播，采用介质折射率
	Vector2D I = incident_ray.m_Dir;
	Vector2D N = segment.m_normal;
	if (n1 == n2) {//射线上的折射率与面元内部折射率相同，射线方向：内部→外部
		N = -N;//内部情况下，扭转面元法向
		n2 = segment.m_refractNOut;
	}
	double dotIN = I * N;
	double critical_angle = asin(n2 / n1);
	double incident_angle = acos(-dotIN);
	if (incident_angle > critical_angle)//大于临界角，不发生透射
		return false;
	RtLbsType ratio = n1 / n2;


	Vector2D I_prep = I - static_cast<RtLbsType>(dotIN) * N; //入射方向沿法向量垂直方向的投影向量
	Vector2D T_prep = ratio * I_prep; //透射方向沿法向量垂直方向的投影分量
	double cos_theta_T = sqrt(1.0 - T_prep.SquaredLength());
	Vector2D T_N = static_cast<RtLbsType>(-1.0 * cos_theta_T) * N;//透射法向沿法向量方向上的分量


	newRay->m_Dir = T_prep + T_N;
	if (isnan(newRay->m_Dir.x)) {
		printf("计算到临界角，请检查错误");
		return false;
	}
		
	*newRay = incident_ray;
	newRay->m_isValid = true;
	newRay->m_limTotl = incident_ray.m_limTotl - 1;
	newRay->m_limTran = incident_ray.m_limTran - 1;
	newRay->m_Ori = inter.m_intersect;
	newRay->m_fRefractiveIndex = n2; /** @brief	进入到"出射介质体中"	*/
	newRay->m_fMin = incident_ray.m_fMin;
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	newRay->m_bsplit = incident_ray.m_bsplit; 
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;

	//构建射线树节点
	node->m_isValid = true;
	node->m_type = tranType;
	node->m_depth = layer;
	node->m_t = newRay->m_fMax;
	node->m_point = inter.m_intersect;
	node->m_segmentId = inter.m_segmentId;
	node->m_wedgeId = inter.m_wedgeId;
	node->m_nextRay = *newRay;

	return true;
}

HOST_DEVICE_FUNC bool GenerateEmpiricalTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay, TreeNodeGPU* node, int layer)
{
	if (!inter.m_propagationProperty.m_hasEmpiricalTransmission)				//若无经验透射属性，则不产生经验透射属性
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	PATHNODETYPE tranType = NODE_ETRANIN;
	if (incident_ray.m_fRefractiveIndex == segment.m_refractN) {
		tranType = NODE_ETRANOUT;
	}
	*newRay = incident_ray;
	newRay->m_isValid = true;
	newRay->m_Ori = inter.m_intersect;
	newRay->m_fMin = incident_ray.m_fMin;
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//叠加传播距离
	newRay->m_bsplit = incident_ray.m_bsplit;
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;

	//构建射线树节点
	node->m_isValid = true;
	node->m_type = tranType;
	node->m_depth = layer;
	node->m_t = newRay->m_fMax;
	node->m_point = inter.m_intersect;
	node->m_segmentId = inter.m_segmentId;
	node->m_wedgeId = inter.m_wedgeId;
	node->m_nextRay = *newRay;

	return true;
}

