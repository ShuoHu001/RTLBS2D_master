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
	GeneralSource* m_fatherSource;			/** @brief	父广义源	*/
	PathNode m_originPathNode;				/** @brief	从pathNode中来源的本体	*/
	GeneralSource* m_replaceValidSource;	/** @brief	可以进行平替的指针，若当前广义源即将被释放，可进行平替的对象指针，在消除重复广义源时候会用到	*/

public:
	GeneralSource();
	GeneralSource(const GeneralSource& s);
	~GeneralSource();
	GeneralSource& operator = (const GeneralSource& s);
	bool IsValid() const;
	void NormalizedWeight(RtLbsType maxWeight);									//归一化权重
	void UpdateEvenPhiValue();													//更新均匀角度值
	void Output2File(std::ofstream& stream) const;								//输出广义源至文件中
	std::string ToString() const;												//输出为字符串
	size_t GetHash() const;														//获取Hash值，唯一标识符
};

//是否有有效的AOA解集
inline bool HasValidAOASolution(const GeneralSource* gs1, const GeneralSource* gs2, const Scene* scene) {
	RtLbsType theta1 = gs1->m_sensorData.m_phi;
	RtLbsType theta2 = gs2->m_sensorData.m_phi;
	RtLbsType x1 = gs1->m_position.x;
	RtLbsType x2 = gs2->m_position.x;
	RtLbsType y1 = gs1->m_position.y;
	RtLbsType y2 = gs2->m_position.y;

	if (std::abs(theta1 - theta2) < EPSILON) {			//若角度相等，不会有解
		return false;
	}
	RtLbsType sinTheta1 = sin(theta1);
	RtLbsType cosTheta1 = cos(theta1);
	RtLbsType sinTheta2 = sin(theta2);
	RtLbsType cosTheta2 = cos(theta2);
	RtLbsType sinTheta12 = sin(theta1 - theta2);

	RtLbsType x = ((x1 * sinTheta1 - y1 * cosTheta1) * cosTheta2 - (x2 * sinTheta2 - y2 * cosTheta2) * cosTheta1) / sinTheta12;
	RtLbsType y = ((x1 * sinTheta1 - y1 * cosTheta1) * sinTheta2 - (x2 * sinTheta2 - y2 * cosTheta2) * sinTheta1) / sinTheta12;

	Point2D p(x, y);									//所求解的交点

	//判定准则-1 是否超出边界
	if (!scene->m_bbox.IsContainPoint(p)) {
		return false;
	}

	//判定准则-2 是否处于环境的无效位置
	if (!scene->IsValidPoint(p)) {
		return false;
	}

	//判定准则-3 在满足AOA规则的条件下是否满足路径碰撞规则
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

//是否是有效的TDOA解集
inline bool HasValidTDOASolution(const GeneralSource* gs1, const GeneralSource* gs2, const Scene* scene) {

}

//是否是有效的TOA解集
inline bool HasValidTOASolution(const GeneralSource* gs1, const GeneralSource* gs2, const Scene* scene) {

}

#endif
