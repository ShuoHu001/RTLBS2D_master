#ifndef RTLBS_RTRESULTSIMILARITY
#define RTLBS_RTRESULTSIMILARITY

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point2d.h"
#include "result/pathinfo.h"
#include "result/raytracingresult.h"

class ReceiverInfo {
public:
	bool m_isValid;								/** @brief	�Ƿ���Ч	*/
	Point2D m_point;							/** @brief	���յ�����	*/
	std::vector<PathInfo> m_multipathInfo;		/** @brief	��ǰ��Ķྶ��Ϣ�����յ�һ���չ���35dB�����	*/
	std::vector<ReceiverInfo*> m_similarities;	/** @brief	���Ƽ���	*/
	std::vector<std::pair<RtLbsType, RtLbsType>> m_phiDistanceInfo;			/** @brief	�ǶȾ���ԣ��Ƕ�-����	*/
	RtLbsType m_maxDistance;					/** @brief	���Ƽ��ϵ���Զ����	*/
	RtLbsType m_meanDistance;					/** @brief	���Ƽ��ϵ�ƽ������	*/
private:
	std::unordered_map<size_t, PathInfo*> pathInfoMap;			/** @brief	�ṩ·����ϢHashӳ��	*/

public:
	ReceiverInfo();
	ReceiverInfo(const RaytracingResult& rtResult);
	~ReceiverInfo();
	bool CanAddToSimilarities(ReceiverInfo* info);			//�Ƿ��ܹ����뵽���ƶȼ�����
	void Init(const RaytracingResult& rtResult);			//��ʼ��
	void UpdateSimilaritiesDistance();						//�������ƶȾ���
	void GetDistanceByPhi(RtLbsType phi, RtLbsType& maxDistance, RtLbsType& meanDistance) const;			//����AOA�Ƕȷ���������ƾ���
	void Write2File(std::ofstream& stream);					//д�����ļ���
};

#endif
