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
	bool m_isValid;							/** @brief	广义源是否有效	*/
	PATHNODETYPE m_type;					/** @brief	广义源类型	*/
	uint16_t m_depth;						/** @brief	广义源深度	*/
	SensorData m_sensorData;				/** @brief	广义源对应的传感器数据	*/
	int m_wCount;							/** @brief	广义源计数权重	*/
	RtLbsType m_weight;						/** @brief	广义源权重	*/
	Point2D m_position;						/** @brief	广义源的位置	*/
	Point2D m_nodePosition;					/** @brief	产生广义源的路径节点坐标	*/
	Segment2D* m_segment;					/** @brief	广义源所在平面	*/
	Wedge2D* m_wedge;						/** @brief	广义源所在棱劈	*/
	int m_phiRepeatCount;					/** @brief	广义源角度重复的数量，默认为1 表示不重复	*/
	PathNode m_originPathNode;				/** @brief	从pathNode中来源的本体	*/

public:
	GeneralSource();
	GeneralSource(const GeneralSource& s);
	~GeneralSource();
	GeneralSource& operator = (const GeneralSource& s);
	bool IsValid() const;														//是否有效
	void NormalizedWeight(RtLbsType maxWeight);									//归一化权重
	void UpdateEvenPhiValue();													//更新均匀角度值
	void Output2File(std::ofstream& stream) const;								//输出广义源至文件中
	std::string ToString() const;												//输出为字符串
	size_t GetHash() const;														//获取Hash值，唯一标识符
};

//功率从大到小顺序
inline bool ComparedByPower_GeneralSource(const GeneralSource* gs1, const GeneralSource* gs2) {
	return gs1->m_sensorData.m_power > gs2->m_sensorData.m_power;
}

//计数权重从大到小顺序
inline bool ComparedByWCount_GeneralSource(const GeneralSource* gs1, const GeneralSource* gs2) {
	return gs1->m_wCount > gs2->m_wCount;
}

#endif
