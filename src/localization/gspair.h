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
	int m_clusterId;								/** @brief	聚类簇ID	*/
	int m_clusterSize;								/** @brief	聚类簇大小	*/
	GeneralSource* m_gs1;
	GeneralSource* m_gs2;
	GeneralSource* m_gsRef;
	GSPairCluster* m_belongingPairCluster;			/** @brief	所属的广义源集合	*/
	Point2D m_targetSolution;
	RtLbsType m_phiResidual;
	RtLbsType m_timeResidual;
	RtLbsType m_timeDiffResidual;
	RtLbsType m_powerResidual;
	RtLbsType m_powerDiffResidual;
	RtLbsType m_angularSpreadResidual;			/** @brief	角度扩展残差	*/
	RtLbsType m_delaySpreadResidual;			/** @brief	延迟扩展残差	*/
	RtLbsType m_weight;							/** @brief	权重	，综合性的权重*/
	int m_nullDataNum;							/** @brief	空数据数量，多数据定位时不满足的数据数量	*/

public:
	GSPair();
	GSPair(GeneralSource* gs1, GeneralSource* gs2);
	GSPair(GeneralSource* gsRef, GeneralSource* gs1, GeneralSource* gs2);
	GSPair(const GSPair& pair);
	~GSPair();
	GSPair& operator = (const GSPair& pair);
	RtLbsType DistanceTo(const GSPair& pair);																						//计算与其他pair之间的距离（解集中的距离）
	void UpdateResidual_AOA(RtLbsType mean_r_phi, RtLbsType mean_r_powerDiff);														//更新权重AOA
	void UpdateResidual_TOA(RtLbsType mean_r_time, RtLbsType mean_r_power);															//更新权重TOA
	void UpdateResidual_TDOA(RtLbsType mean_r_timeDiff, RtLbsType mean_r_powerDiff);												//更新权重TDOA
	void UpdateResidual_AOATOA(RtLbsType mean_r_phi, RtLbsType mean_r_time, RtLbsType mean_r_power);								//更新权重AOATOA
	void UpdateResidual_AOATDOA(RtLbsType mean_r_phi, RtLbsType mean_r_timeDiff, RtLbsType mean_r_powerDiff);						//跟新权重AOATDOA
	void NormalizedWeight(RtLbsType max_weight);																				//计算归一化权重
	bool HasValidAOASolution(const Scene* scene);																				//验证解的有效性，仅在几何上进行验证
	bool HasValidTOASolution(const Scene* scene);		//是否有TOA解
	bool HasValidTOASolution(const Point2D& p, const Scene* scene);		//是否有TOA对称解
	bool HasValidAOATOASolution(const Scene* scene);
	bool HasValidAOATDOASolution(const Scene* scene);
	bool HasValidTDOASolution(const Scene* scene);
	void CalculateSinglePairResidual();																												//计算单个pair组合的残差
	void CalNormalizedWeightAndUpdate_AOA(RtLbsType max_r_phi, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);										//计算部分归一化权重值并更新广义源权重-AOA型
	void CalNormalizedWeightAndUpdate_TOA(RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w, int max_clusterNum);											//计算部分归一化权重值并更新广义源权重-TOA型
	void CalNormalizedWeightAndUpdate_AOATOA(RtLbsType max_r_phi, RtLbsType max_r_time, RtLbsType max_r_power, const WeightFactor& w, int max_clusterNum);					//计算部分归一化权重值并更新广义源权重-AOATOA型
	void CalNormalizedWeightAndUpdate_TDOA(RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);									//计算部分归一化权重值并更新广义源权重-TDOA型
	void CalNormalizedWeightAndUpdate_AOATDOA(RtLbsType max_r_phi, RtLbsType max_r_timeDiff, RtLbsType max_r_powerDiff, const WeightFactor& w, int max_clusterNum);		//计算部分归一化权重值并更新广义源权重-AOA/TDOA型
	
private:
	bool _calAOASolution();
	bool _calTOASolution(const BBox2D& bbox);
	bool _calTDOASolution(const BBox2D& bbox);
	bool _calAOATOASolution(const BBox2D& bbox);
	bool _calAOATDOASolution(const BBox2D& bbox);
	bool _judgementRules(const Scene* scene);								//初步定位解判定准则

};

inline bool PreCheckGSPairValidation_TDOA(GeneralSource* gsRef, GeneralSource* gs1, GeneralSource* gs2) {
	//若gs1或gs2中与gsref点位相同，则无效
	if (gsRef->m_position == gs1->m_position ||
		gsRef->m_position == gs2->m_position ||
		gs1->m_position == gs2->m_position) {
		return false;
	}

	//若传感器数据相同，则解无效
	if (gsRef->m_sensorData.m_id == gs1->m_sensorData.m_id ||
		gsRef->m_sensorData.m_id == gs2->m_sensorData.m_id ||
		gs1->m_sensorData.m_id == gs2->m_sensorData.m_id) {
		return false;
	}

	//根据时延差来判定是否为有效的广义源组合
	RtLbsType timeDiff_data1 = gs1->m_sensorData.m_timeDiff;
	RtLbsType timeDiff_data2 = gs2->m_sensorData.m_timeDiff;
	RtLbsType time_refNode = gsRef->m_originPathNode.m_ft;
	RtLbsType time_dataNode1 = gs1->m_originPathNode.m_ft;
	RtLbsType time_dataNode2 = gs2->m_originPathNode.m_ft;
	RtLbsType flag_data1 = time_dataNode1 * (time_dataNode1 - time_refNode);
	RtLbsType flag_data2 = time_dataNode2 * (time_dataNode2 - time_refNode);
	if (flag_data1 < 0 || flag_data2 < 0) {									//保证时延差的符号与节点坐标传播距离差的符号相同
		return false;
	}
	return true;
}

#endif
