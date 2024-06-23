#ifndef RTLBS_GENERALIZEDSOURCE
#define RTLBS_GENERALIZEDSOURCE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "tree/pathnode.h"
#include "geometry/segment2d.h"
#include "geometry/wedge2d.h"
#include "geometry/scene.h"
#include "equipment/sensordata.h"
#include "equipment/sensor.h"
#include "tree/pathnode.h"

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
	GeneralSource* m_fatherSource;			/** @brief	������Դ	*/
	PathNode m_originPathNode;				/** @brief	��pathNode����Դ�ı���	*/
	GeneralSource* m_replaceValidSource;	/** @brief	���Խ���ƽ���ָ�룬����ǰ����Դ�������ͷţ��ɽ���ƽ��Ķ���ָ�룬�������ظ�����Դʱ����õ�	*/

public:
	GeneralSource();
	GeneralSource(const GeneralSource& s);
	~GeneralSource();
	GeneralSource& operator = (const GeneralSource& s);
	bool IsValid() const;
	void NormalizedWeight(RtLbsType maxWeight);									//��һ��Ȩ��
	void UpdateEvenPhiValue();													//���¾��ȽǶ�ֵ
	void Output2File(std::ofstream& stream) const;								//�������Դ���ļ���
	std::string ToString() const;												//���Ϊ�ַ���
	size_t GetHash() const;														//��ȡHashֵ��Ψһ��ʶ��
};

//�Ƿ�����Ч��AOA�⼯
inline bool HasValidAOASolution(const GeneralSource* gs1, const GeneralSource* gs2, const Scene* scene) {
	RtLbsType theta1 = gs1->m_sensorData.m_phi;
	RtLbsType theta2 = gs2->m_sensorData.m_phi;
	RtLbsType x1 = gs1->m_position.x;
	RtLbsType x2 = gs2->m_position.x;
	RtLbsType y1 = gs1->m_position.y;
	RtLbsType y2 = gs2->m_position.y;

	if (std::abs(theta1 - theta2) < EPSILON) {			//���Ƕ���ȣ������н�
		return false;
	}
	RtLbsType sinTheta1 = sin(theta1);
	RtLbsType cosTheta1 = cos(theta1);
	RtLbsType sinTheta2 = sin(theta2);
	RtLbsType cosTheta2 = cos(theta2);
	RtLbsType sinTheta12 = sin(theta1 - theta2);

	RtLbsType x = ((x1 * sinTheta1 - y1 * cosTheta1) * cosTheta2 - (x2 * sinTheta2 - y2 * cosTheta2) * cosTheta1) / sinTheta12;
	RtLbsType y = ((x1 * sinTheta1 - y1 * cosTheta1) * sinTheta2 - (x2 * sinTheta2 - y2 * cosTheta2) * sinTheta1) / sinTheta12;

	Point2D p(x, y);									//�����Ľ���

	//�ж�׼��-1 �Ƿ񳬳��߽�
	if (!scene->m_bbox.IsContainPoint(p)) {
		return false;
	}

	//�ж�׼��-2 �Ƿ��ڻ�������Чλ��
	if (!scene->IsValidPoint(p)) {
		return false;
	}

	//�ж�׼��-3 ������AOA������������Ƿ�����·����ײ����
	const Point2D& s1 = gs1->m_nodePosition;
	const Point2D& s2 = gs2->m_nodePosition;

	Segment2D segment1(p, s1);
	Segment2D segment2(p, s2);

	if (scene->GetIntersect(segment1, nullptr)) {
		return false;
	}
	if (scene->GetIntersect(segment1, nullptr)) {
		return false;
	}
	return true;
}

//�Ƿ�����Ч��TDOA�⼯
inline bool HasValidTDOASolution(const GeneralSource* gs1, const GeneralSource* gs2, const Scene* scene) {

}

//�Ƿ�����Ч��TOA�⼯
inline bool HasValidTOASolution(const GeneralSource* gs1, const GeneralSource* gs2, const Scene* scene) {

}

#endif
