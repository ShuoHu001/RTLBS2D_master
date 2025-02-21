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
	bool m_isValid;								/** @brief	�Ƿ���Ч	*/
	Point2D m_point;							/** @brief	���յ�����	*/
	std::vector<PathInfo> m_multipathInfo;		/** @brief	��ǰ��Ķྶ��Ϣ�����յ�һ���չ���35dB�����	*/
	std::vector<ReceiverInfo*> m_similarities;	/** @brief	���Ƽ���	*/
	std::vector<AOA_Distance> m_AOADistanceInfo;			/** @brief	�ǶȾ���ԣ��Ƕ�-����	*/
	std::vector<TOA_Distance> m_TOADistanceInfo;		/** @brief	ʱ�Ӿ���ԣ�ʱ��-����	*/
	std::vector<AOA_TDOA_Distance> m_AOA_TDOADistanceInfo;
	RtLbsType m_maxDistance;					/** @brief	���Ƽ��ϵ���Զ����	*/
	RtLbsType m_meanDistance;					/** @brief	���Ƽ��ϵ�ƽ������	*/
	std::unordered_map<size_t, PathInfo> pathInfoMap;			/** @brief	�ṩ·����ϢHashӳ��	*/

public:
	ReceiverInfo();
	ReceiverInfo(const RaytracingResult& rtResult);
	~ReceiverInfo();
	bool CanAddToSimilarities_AOA(const ReceiverInfo& info);			//�Ƿ��ܹ����뵽���ƶȼ�����-AOA��λ�㷨
	bool CanAddToSimilarities_TOA(const ReceiverInfo& info);			//�Ƿ��ܹ����뵽���ƶȼ�����-TOA��λ�㷨
	bool CanAddToSimilarities_AOATDOA(const ReceiverInfo& info);		//�Ƿ��ܹ����뵽���ƶȼ�����-AOATDOA��λ�㷨
	void Init(const RaytracingResult& rtResult);			//��ʼ��
	void UpdateSimilaritiesDistance_AOA();						//�������ƶȾ���_AOA
	void UpdateSimilaritiesDistance_TOA();						//�������ƶȾ���_TOA
	void UpdateSimilaritiesDistance_AOATDOA();					//�������ƶȾ���_AOATDOA
	void GetDistanceByAOA(RtLbsType phi, RtLbsType& maxDistance, RtLbsType& meanDistance) const;			//����AOA�Ƕȷ���������ƾ���
	void GetDistanceByTOA(RtLbsType delay, RtLbsType& maxDistance, RtLbsType& meanDistance) const;			//����TOA����������ƾ���
	void GetDistanceByAOATDOA(RtLbsType phi, RtLbsType timeDiff, RtLbsType& maxDistance, RtLbsType& meanDistance) const;	//����AOA��TDOA����������ƾ���
	void Write2File(std::ofstream& stream);					//д�����ļ���
};


#endif
