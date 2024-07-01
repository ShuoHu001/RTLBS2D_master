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
#include "tree/raypath.h"
#include "tree/raypath3d.h"

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
	bool CalTDOAParameters_SPSTMD(const Point2D& targetPoint, const Scene* scene, RtLbsType freq, const std::vector<Complex>& tranFunction, RtLbsType& delay, RtLbsType& power) const;			//����TDOA��λģʽ�Ĳ���ֵ-��վ��λģʽ
	bool CalTDOAParameters_MPSTSD(const Point2D& targetPoint, const Scene* scene) const;																										//��վ��λģʽ
	bool IsValid() const;														//�Ƿ���Ч
	void NormalizedWeight(RtLbsType maxWeight);									//��һ��Ȩ��
	void UpdateEvenPhiValue();													//���¾��ȽǶ�ֵ
	void Output2File(std::ofstream& stream) const;								//�������Դ���ļ���
	std::string ToString() const;												//���Ϊ�ַ���
	size_t GetHash() const;														//��ȡHashֵ��Ψһ��ʶ��
};


#endif
