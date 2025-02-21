#ifndef RTLBS_RTRESULTSIMILARITY
#define RTLBS_RTRESULTSIMILARITY

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point2d.h"
#include "result/pathinfo.h"
#include "result/raytracingresult.h"

struct AOA_Distance {
	RtLbsType angle;
	RtLbsType distance;
};

struct TOA_Distance {
	RtLbsType time;
	RtLbsType distance;
};

struct AOA_TDOA_Distance {
	RtLbsType angle;
	RtLbsType timeDifference;
	RtLbsType distance;
};

class ReceiverInfo {
public:
	bool m_isValid;								/** @brief	是否有效	*/
	Point2D m_point;							/** @brief	接收点坐标	*/
	std::vector<PathInfo> m_multipathInfo;		/** @brief	当前点的多径信息，按照第一接收功率35dB后进行	*/
	std::vector<ReceiverInfo*> m_similarities;	/** @brief	相似集合	*/
	std::vector<AOA_Distance> m_AOADistanceInfo;			/** @brief	角度距离对，角度-距离	*/
	std::vector<TOA_Distance> m_TOADistanceInfo;		/** @brief	时延距离对，时延-距离	*/
	std::vector<AOA_TDOA_Distance> m_AOA_TDOADistanceInfo;
	RtLbsType m_maxDistance;					/** @brief	相似集合的最远距离	*/
	RtLbsType m_meanDistance;					/** @brief	相似集合的平均距离	*/
	std::unordered_map<size_t, PathInfo> pathInfoMap;			/** @brief	提供路径信息Hash映射	*/

public:
	ReceiverInfo();
	ReceiverInfo(const RaytracingResult& rtResult);
	~ReceiverInfo();
	bool CanAddToSimilarities_AOA(const ReceiverInfo& info);			//是否能够加入到相似度集合中-AOA定位算法
	bool CanAddToSimilarities_TOA(const ReceiverInfo& info);			//是否能够加入到相似度集合中-TOA定位算法
	bool CanAddToSimilarities_AOATDOA(const ReceiverInfo& info);		//是否能够加入到相似度集合中-AOATDOA定位算法
	void Init(const RaytracingResult& rtResult);			//初始化
	void UpdateSimilaritiesDistance_AOA();						//更新相似度距离_AOA
	void UpdateSimilaritiesDistance_TOA();						//更新相似度距离_TOA
	void UpdateSimilaritiesDistance_AOATDOA();					//更新相似度距离_AOATDOA
	void GetDistanceByAOA(RtLbsType phi, RtLbsType& maxDistance, RtLbsType& meanDistance) const;			//基于AOA角度返回最大相似距离
	void GetDistanceByTOA(RtLbsType delay, RtLbsType& maxDistance, RtLbsType& meanDistance) const;			//基于TOA返回最大相似距离
	void GetDistanceByAOATDOA(RtLbsType phi, RtLbsType timeDiff, RtLbsType& maxDistance, RtLbsType& meanDistance) const;	//基于AOA和TDOA返回最大相似距离
	void Write2File(std::ofstream& stream);					//写入至文件中
};


#endif
