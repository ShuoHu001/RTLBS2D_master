#ifndef RTLBS_GSPAIR
#define RTLBS_GSPAIR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/bbox2d.h"
#include "geometry/scene.h"
#include "generalsource.h"
#include "localization/tdoa/tdoasolver.h"

class GSPair {
public:
	bool m_isValid;
	int m_clusterId;								/** @brief	�����ID	*/
	int m_clusterSize;								/** @brief	����ش�С	*/
	GeneralSource* m_gs1;
	GeneralSource* m_gs2;
	GeneralSource* m_gsRef;
	Point2D m_targetSolution;
	RtLbsType m_phiResidual;
	RtLbsType m_timeResidual;
	RtLbsType m_timeDiffResidual;
	RtLbsType m_powerDiffResidual;
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
	void NormalizedWeight(RtLbsType max_weight);																				//�����һ��Ȩ��
	bool HasValidAOASolution(const Scene* scene);																				//��֤�����Ч�ԣ����ڼ����Ͻ�����֤
	bool HasValidTOASolution(const Scene* scene);
	bool HasValidTDOASolution_SPSTMD(const Scene* scene, RtLbsType freq, const std::vector<Complex>& tranFunction);									//��֤TDOA�㷨����Ч�� ��Դ�����ݶ�λ
	bool HasValidTDOASolution_MPSTSD(const Scene* scene);																							//��֤TDOA�㷨����Ч�� ��Դ�����ݶ�λ
	void CalculateSinglePairResidual();																												//���㵥��pair��ϵĲв�
	void CalNormalizedWeightAndUpdate_AOA(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);										//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-AOA��
	void CalNormalizedWeightAndUpdate_TDOA(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);									//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-TDOA��
	void CalNormalizedWeightAndUpdate_AOA_TDOA(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);		//���㲿�ֹ�һ��Ȩ��ֵ�����¹���ԴȨ��-AOA/TDOA��
	
private:
	bool _calAOASolution();
	bool _calTOASolution();
	bool _calTDOASolution();

};

#endif
