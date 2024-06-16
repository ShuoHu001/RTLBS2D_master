#ifndef RTLBS_GSPAIR
#define RTLBS_GSPAIR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/bbox2d.h"
#include "geometry/scene.h"
#include "generalsource.h"

class GSPair {
public:
	bool m_isValid;
	GeneralSource* m_gs1;
	GeneralSource* m_gs2;
	GeneralSource* m_gsRef;
	Point2D m_targetSolution;
	RtLbsType m_phiResidual;
	RtLbsType m_timeResidual;
	RtLbsType m_timeDiffResidual;
	RtLbsType m_powerDiffResidual;
	RtLbsType m_weight;							/** @brief	权重	，综合性的权重*/

public:
	GSPair();
	GSPair(GeneralSource* gs1, GeneralSource* gs2);
	GSPair(GeneralSource* gsRef, GeneralSource* gs1, GeneralSource* gs2);
	GSPair(const GSPair& pair);
	~GSPair();
	GSPair& operator = (const GSPair& pair);
	void NormalizedWeight(RtLbsType max_weight);																				//计算归一化权重
	bool HasValidAOASolution(const Scene* scene);																				//验证解的有效性，仅在几何上进行验证
	bool HasValidTOASolution(const Scene* scene);
	bool HasValidTDOASolution(const Scene* scene);
	void CalNormalizedWeightAndUpdate_AOA(RtLbsType max_r_phi, RtLbsType max_r_powerDiff);										//计算部分归一化权重值并更新广义源权重-AOA型
	void CalNormalizedWeightAndUpdate_TDOA(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff);								//计算部分归一化权重值并更新广义源权重-TDOA型
	void CalNormalizedWeightAndUpdate_AOA_TDOA(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff);		//计算部分归一化权重值并更新广义源权重-AOA/TDOA型
	
private:
	bool _calAOASolution();
	bool _calTOASolution();
	bool _calTDOASolution();

};

#endif
