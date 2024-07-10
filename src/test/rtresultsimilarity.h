#ifndef RTLBS_RTRESULTSIMILARITY
#define RTLBS_RTRESULTSIMILARITY

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/point2d.h"
#include "result/pathinfo.h"
#include "result/raytracingresult.h"

class ReceiverInfo {
public:
	bool m_isValid;								/** @brief	是否有效	*/
	Point2D m_point;							/** @brief	接收点坐标	*/
	std::vector<PathInfo> m_multipathInfo;		/** @brief	当前点的多径信息，按照第一接收功率35dB后进行	*/
	std::vector<ReceiverInfo*> m_similarities;	/** @brief	相似集合	*/
	RtLbsType m_maxDistance;					/** @brief	相似集合的最远距离	*/
	RtLbsType m_meanDistance;					/** @brief	相似集合的平均距离	*/
private:
	std::unordered_map<size_t, PathInfo*> pathInfoMap;			/** @brief	提供路径信息Hash映射	*/

public:
	ReceiverInfo();
	ReceiverInfo(const RaytracingResult& rtResult);
	~ReceiverInfo();
	bool CanAddToSimilarities(ReceiverInfo* info);			//是否能够加入到相似度集合中
	void Init(const RaytracingResult& rtResult);			//初始化
	void UpdateSimilaritiesDistance();						//更新相似度距离
	void Write2File(std::ofstream& stream);					//写入至文件中
};

#endif
