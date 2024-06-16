#ifndef RTLBS_PATHNODEGPU
#define RTLBS_PATHNODEGPU

#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/gpu/ray2dgpu.h"
#include "geometry/point2d.h"
#include "geometry/gpu/Intersection2Dgpu.h"

class PathNodeGPU {

public:
	bool m_isValid;										/** @brief	�Ƿ���Ч	*/
	int m_layer;										/** @brief	�ڵ����ڵĵ�������	*/
	int m_rxId;											/** @brief	�ڵ������rx���	*/
	RtLbsType m_ft;										/** @brief	��������	*/
	Intersection2DGPU m_inter;							/** @brief	�ڵ����ڵĽ�����Ϣ	*/
	

public:
	HOST_DEVICE_FUNC PathNodeGPU();
	HOST_DEVICE_FUNC PathNodeGPU(const PathNodeGPU& node);										//��ֵ���캯��
	HOST_DEVICE_FUNC ~PathNodeGPU() {};
	HOST_DEVICE_FUNC PathNodeGPU(bool isValid, int layer, int rxId, Intersection2DGPU inter);
	HOST_DEVICE_FUNC PathNodeGPU& operator = (const PathNodeGPU& node);//��ֵ���������
};

#endif
