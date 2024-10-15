#include "transmissiongpu.h"

HOST_DEVICE_FUNC bool GenerateTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay) {
	if (!inter.m_propagationProperty.m_hasTransmission)						//����͸�����ԣ��򲻲���͸������
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	PATHNODETYPE tranType = NODE_TRANIN;
	if (incident_ray.m_fRefractiveIndex == segment.m_refractN) {
		tranType = NODE_TRANOUT;
	}
	if (segment.m_refractN == segment.m_refractNOut) {//����Ϊ�����������ʣ����ı䴫������
		*newRay = incident_ray;
		newRay->m_isValid = true;
		newRay->m_Ori = inter.m_intersect;
		newRay->m_fMin = incident_ray.m_fMin;
		newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
		newRay->m_primitiveId = inter.m_segmentId;
		newRay->m_nodeType = tranType;
		newRay->m_prevInterId = prevInterId;
		return true;
	}
	RtLbsType n1, n2;//n1:ǰһ�ֽ��ʵ������ʣ�n2:��һ�ֽ��ʵ�������
	n1 = incident_ray.m_fRefractiveIndex;
	n2 = segment.m_refractN;                                           //Ĭ�����ߴ��ⲿ���ڲ����������ý���������
	Vector2D I = incident_ray.m_Dir;
	Vector2D N = segment.m_normal;
	if (n1 == n2) {//�����ϵ�����������Ԫ�ڲ���������ͬ�����߷����ڲ����ⲿ
		N = -N;//�ڲ�����£�Ťת��Ԫ����
		n2 = segment.m_refractNOut;
	}
	double dotIN = I * N;
	double critical_angle = asin(n2 / n1);
	double incident_angle = acos(-dotIN);
	if (incident_angle > critical_angle)//�����ٽ�ǣ�������͸��
		return false;
	RtLbsType ratio = n1 / n2;


	Vector2D I_prep = I - static_cast<RtLbsType>(dotIN) * N; //���䷽���ط�������ֱ�����ͶӰ����
	Vector2D T_prep = ratio * I_prep; //͸�䷽���ط�������ֱ�����ͶӰ����
	double cos_theta_T = sqrt(1.0 - T_prep.SquaredLength());
	Vector2D T_N = static_cast<RtLbsType>(-1.0 * cos_theta_T) * N;//͸�䷨���ط����������ϵķ���


	newRay->m_Dir = T_prep + T_N;
	if (isnan(newRay->m_Dir.x))
		printf("���㵽�ٽ�ǣ��������");
	*newRay = incident_ray;
	newRay->m_isValid = true;
	newRay->m_limTotl = incident_ray.m_limTotl - 1;
	newRay->m_limTran = incident_ray.m_limTran - 1;
	newRay->m_Ori = inter.m_intersect;
	newRay->m_fRefractiveIndex = n2; /** @brief	���뵽"�����������"	*/
	newRay->m_fMin = incident_ray.m_fMin;
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	newRay->m_bsplit = false; //͸�����߲��߱���������
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;
	return true;
}

HOST_DEVICE_FUNC bool GenerateEmpiricalTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay)
{
	if (!inter.m_propagationProperty.m_hasEmpiricalTransmission)				//���޾���͸�����ԣ��򲻲�������͸������
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
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;
	return true;
}

HOST_DEVICE_FUNC bool GenerateTransmitRayGPU(const Intersection2DGPU& inter, int prevInterId, const Segment2DGPU* segments, Ray2DGPU* newRay, TreeNodeGPU* node, int layer)
{
	if (!inter.m_propagationProperty.m_hasTransmission)						//����͸�����ԣ��򲻲���͸������
		return false;
	const Segment2DGPU& segment = segments[inter.m_segmentId];
	const Ray2DGPU& incident_ray = inter.m_ray;
	if (incident_ray.m_nodeType == NODE_DIFF) {				//����󲻽���͸��
		return false;
	}
	PATHNODETYPE tranType = NODE_TRANIN;
	if (incident_ray.m_fRefractiveIndex == segment.m_refractN) {
		tranType = NODE_TRANOUT;
	}
	if (segment.m_refractN == segment.m_refractNOut) {//����Ϊ�����������ʣ����ı䴫������
		*newRay = incident_ray;
		newRay->m_isValid = true;
		newRay->m_Ori = inter.m_intersect;
		newRay->m_fMin = incident_ray.m_fMin;
		newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
		newRay->m_primitiveId = inter.m_segmentId;
		newRay->m_nodeType = tranType;
		newRay->m_prevInterId = prevInterId;
		return true;
	}
	RtLbsType n1, n2;//n1:ǰһ�ֽ��ʵ������ʣ�n2:��һ�ֽ��ʵ�������
	n1 = incident_ray.m_fRefractiveIndex;
	n2 = segment.m_refractN;                                           //Ĭ�����ߴ��ⲿ���ڲ����������ý���������
	Vector2D I = incident_ray.m_Dir;
	Vector2D N = segment.m_normal;
	if (n1 == n2) {//�����ϵ�����������Ԫ�ڲ���������ͬ�����߷����ڲ����ⲿ
		N = -N;//�ڲ�����£�Ťת��Ԫ����
		n2 = segment.m_refractNOut;
	}
	double dotIN = I * N;
	double critical_angle = asin(n2 / n1);
	double incident_angle = acos(-dotIN);
	if (incident_angle > critical_angle)//�����ٽ�ǣ�������͸��
		return false;
	RtLbsType ratio = n1 / n2;


	Vector2D I_prep = I - static_cast<RtLbsType>(dotIN) * N; //���䷽���ط�������ֱ�����ͶӰ����
	Vector2D T_prep = ratio * I_prep; //͸�䷽���ط�������ֱ�����ͶӰ����
	double cos_theta_T = sqrt(1.0 - T_prep.SquaredLength());
	Vector2D T_N = static_cast<RtLbsType>(-1.0 * cos_theta_T) * N;//͸�䷨���ط����������ϵķ���


	newRay->m_Dir = T_prep + T_N;
	if (isnan(newRay->m_Dir.x)) {
		printf("���㵽�ٽ�ǣ��������");
		return false;
	}
		
	*newRay = incident_ray;
	newRay->m_isValid = true;
	newRay->m_limTotl = incident_ray.m_limTotl - 1;
	newRay->m_limTran = incident_ray.m_limTran - 1;
	newRay->m_Ori = inter.m_intersect;
	newRay->m_fRefractiveIndex = n2; /** @brief	���뵽"�����������"	*/
	newRay->m_fMin = incident_ray.m_fMin;
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	newRay->m_bsplit = incident_ray.m_bsplit; 
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;

	//�����������ڵ�
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
	if (!inter.m_propagationProperty.m_hasEmpiricalTransmission)				//���޾���͸�����ԣ��򲻲�������͸������
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
	newRay->m_fMax = incident_ray.m_fMax + inter.m_ft;//���Ӵ�������
	newRay->m_bsplit = incident_ray.m_bsplit;
	newRay->m_primitiveId = inter.m_segmentId;
	newRay->m_nodeType = tranType;
	newRay->m_prevInterId = prevInterId;

	//�����������ڵ�
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

