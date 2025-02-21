#ifndef RTLBS_GSPAIR
#define RTLBS_GSPAIR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "geometry/bbox2d.h"
#include "scene/scene.h"
#include "generalsource.h"
#include "localization/tdoa/tdoasolver.h"
#include "localization/aoa_toa/aoa_toa_solver.h"
#include "localization/toa/toa_solver.h"
#include "localization/aoa_tdoa/aoa_tdoa_solver.h"

class GSPairCluster;

class GSPair {
public:
	bool m_isValid;
	int m_clusterId;								/** @brief	�����ID	*/
	int m_clusterSize;								/** @brief	����ش�С	*/
	GeneralSource* m_gs1;
	GeneralSource* m_gs2;
	GeneralSource* m_gsRef;
	GSPairCluster* m_belongingPairCluster;			/** @brief	�����Ĺ���Դ����	*/
	Point2D m_targetSolution;
	RtLbsType m_phiResidual;
	RtLbsType m_timeResidual;
	RtLbsType m_timeDiffResidual;
	RtLbsType m_powerResidual;
	RtLbsType m_powerDiffResidual;
	RtLbsType m_angularSpreadResidual;			/** @brief	�Ƕ���չ�в�	*/
	RtLbsType m_delaySpreadResidual;			/** @brief	�ӳ���չ�в�	*/
	RtLbsType m_weight;							/** @brief	Ȩ��	���ۺ��Ե�Ȩ��*/
	int m_nullDataNum;							/** @brief	�����������������ݶ�λʱ���������������	*/

public:
	GSPair();
	GSPair(GeneralSource* gs1, GeneralSource* gs2);
	GSPair(GeneralSource* gsRef, GeneralSource* gs1, GeneralSource* gs2);
	GSPair(const GSPair& pair);
	~GSPair();
	GSPair& operator = (const GSPair& pair);
	RtLbsType DistanceTo(const GSPair& pair);																						//����������pair֮��ľ��루�⼯�еľ��룩
	void UpdateResidual_AOA(RtLbsType mean_r_phi, RtLbsType mean_r_powerDiff);														//����Ȩ��AOA
	void UpdateResidual_TOA(RtLbsType mean_r_time, RtLbsType mean_r_power);															//����Ȩ��TOA
	void UpdateResidual_TDOA(RtLbsType mean_r_timeDiff, RtLbsType mean_r_powerDiff);												//����Ȩ��TDOA
	void UpdateResidual_AOATOA(RtLbsType mean_r_phi, RtLbsType mean_r_time, RtLbsType mean_r_power);								//����Ȩ��AOATOA
	void UpdateResidual_AOATDOA(RtLbsType mean_r_phi, RtLbsType mean_r_timeDiff, RtLbsType mean_r_powerDiff);						//����Ȩ��AOATDOA
	void NormalizedWeight(RtLbsType max_weight);																				//�����һ��Ȩ��
	bool HasValidAOASolution(const Scene* scene);																				//��֤�����Ч�ԣ����ڼ����Ͻ�����֤
	bool HasValidTOASolution(const Scene* scene);		//�Ƿ���TOA��
	bool HasValidTOASolution(const Point2D& p, const Scene* scene);		//�Ƿ���TOA�Գƽ�
	bool HasValidAOATOASolution(const Scene* scene);
	bool HasValidAOATDOASolution(const Scene* scene);
	bool HasValidTDOASolution(const Scene* scene);
	void CalculateSinglePairResidual();																												//���㵥��pair��ϵĲв�
	void CalNormalizedWeightAndUpdate_AOA(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);										//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-AOA��
	void CalNormalizedWeightAndUpdate_TOA(RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w, int max_clusterNum);											//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-TOA��
	void CalNormalizedWeightAndUpdate_AOATOA(RtLbsType max_r_phi, RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w, int max_clusterNum);					//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-AOATOA��
	void CalNormalizedWeightAndUpdate_TDOA(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);									//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-TDOA��
	void CalNormalizedWeightAndUpdate_AOATDOA(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);		//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-AOA/TDOA��
	
private:
	bool _calAOASolution();
	bool _calTOASolution(const BBox2D& bbox);
	bool _calTDOASolution(const BBox2D& bbox);
	bool _calAOATOASolution(const BBox2D& bbox);
	bool _calAOATDOASolution(const BBox2D& bbox);
	bool _judgementRules(const Scene* scene);								//������λ���ж�׼��

};

inline bool PreCheckGSPairValidation_TDOA(GeneralSource* gsRef, GeneralSource* gs1, GeneralSource* gs2) {
	//��gs1��gs2����gsref��λ��ͬ������Ч
	if (gsRef->m_position == gs1->m_position ||
		gsRef->m_position == gs2->m_position ||
		gs1->m_position == gs2->m_position) {
		return false;
	}

	//��������������ͬ�������Ч
	if (gsRef->m_sensorData.m_id == gs1->m_sensorData.m_id ||
		gsRef->m_sensorData.m_id == gs2->m_sensorData.m_id ||
		gs1->m_sensorData.m_id == gs2->m_sensorData.m_id) {
		return false;
	}

	//����ʱ�Ӳ����ж��Ƿ�Ϊ��Ч�Ĺ���Դ���
	RtLbsType timeDiff_data1 = gs1->m_sensorData.m_timeDiff;
	RtLbsType timeDiff_data2 = gs2->m_sensorData.m_timeDiff;
	RtLbsType time_refNode = gsRef->m_originPathNode.m_ft;
	RtLbsType time_dataNode1 = gs1->m_originPathNode.m_ft;
	RtLbsType time_dataNode2 = gs2->m_originPathNode.m_ft;
	RtLbsType flag_data1 = time_dataNode1 * (time_dataNode1 - time_refNode);
	RtLbsType flag_data2 = time_dataNode2 * (time_dataNode2 - time_refNode);
	if (flag_data1 < 0 || flag_data2 < 0) {									//��֤ʱ�Ӳ�ķ�����ڵ����괫�������ķ�����ͬ
		return false;
	}
	return true;
}

#endif
