#ifndef RTLBS_GENERALIZEDSOURCE
#define RTLBS_GENERALIZEDSOURCE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "radiowave/raypath/pathnode.h"
#include "geometry/segment2d.h"
#include "geometry/wedge2d.h"
#include "scene/scene.h"
#include "equipment/sensor/sensordata.h"
#include "equipment/sensor/sensor.h"
#include "radiowave/raypath/pathnode.h"
#include "radiowave/raypath/raypath.h"
#include "radiowave/raypath/raypath3d.h"
#include "efield/calraypathefield.h"

class GeneralSource {
public:
	bool m_isValid;							/** @brief	����Դ�Ƿ���Ч	*/
	PATHNODETYPE m_type;					/** @brief	����Դ����	*/
	uint16_t m_depth;						/** @brief	����Դ���	*/
	SensorData m_sensorData;				/** @brief	����Դ��Ӧ�Ĵ���������	*/
	int m_wCount;							/** @brief	����Դ����Ȩ��	*/
	RtLbsType m_weight;						/** @brief	����ԴȨ��	*/
	Point2D m_position;						/** @brief	����Դ��λ��	*/
	Point2D m_nodePosition;					/** @brief	��������Դ��·���ڵ�����	*/
	Segment2D* m_segment;					/** @brief	����Դ����ƽ��	*/
	Wedge2D* m_wedge;						/** @brief	����Դ��������	*/
	int m_phiRepeatCount;					/** @brief	����Դ�Ƕ��ظ���������Ĭ��Ϊ1 ��ʾ���ظ�	*/
	PathNode m_originPathNode;				/** @brief	��pathNode����Դ�ı���	*/

public:
	GeneralSource();
	GeneralSource(const GeneralSource& s);
	~GeneralSource();
	GeneralSource& operator = (const GeneralSource& s);
	bool IsValid() const;														//�Ƿ���Ч
	void NormalizedWeight(RtLbsType maxWeight);									//��һ��Ȩ��
	void UpdateEvenPhiValue();													//���¾��ȽǶ�ֵ
	void Output2File(std::ofstream& stream) const;								//�������Դ���ļ���
	std::string ToString() const;												//���Ϊ�ַ���
	size_t GetHash() const;														//��ȡHashֵ��Ψһ��ʶ��
};

//���ʴӴ�С˳��
inline bool ComparedByPower_GeneralSource(const GeneralSource* gs1, const GeneralSource* gs2) {
	return gs1->m_sensorData.m_power > gs2->m_sensorData.m_power;
}

//����Ȩ�شӴ�С˳��
inline bool ComparedByWCount_GeneralSource(const GeneralSource* gs1, const GeneralSource* gs2) {
	return gs1->m_wCount > gs2->m_wCount;
}

#endif
