#include "rtresultsimilarity.h"

ReceiverInfo::ReceiverInfo()
	: m_isValid(true)
	, m_maxDistance(0.0)
	, m_meanDistance(0.0)
{
}

ReceiverInfo::ReceiverInfo(const RaytracingResult& rtResult)
	: m_isValid(true)
	, m_maxDistance(0.0)
	, m_meanDistance(0.0)
{
	m_point = rtResult.m_receiver->GetPosition2D();
	if (rtResult.m_multipathInfo.size() == 0) {
		m_isValid = false;
		return;
	}
	std::vector<PathInfo> infos = rtResult.m_multipathInfo;
	std::sort(infos.begin(), infos.end(), ComparedByPower_PathInfo);			//按照功率大小进行排序
	RtLbsType maxPower = infos.front().m_power;

	//抽取峰值后35dB后作为有效多径
	for (auto& info : infos) {
		if (info.m_rayPathType == RAYPATH_TERRAIN_REFLECTION || info.m_rayPathType == RAYPATH_TERRAIN_DIFFRACTION) {
			continue;
		}
		if ((maxPower - info.m_power) < 40) {
			m_multipathInfo.push_back(info);
			size_t hashcode = info.GetHash();
			pathInfoMap[hashcode] = info;
		}
	}
}

ReceiverInfo::~ReceiverInfo()
{
}

bool ReceiverInfo::CanAddToSimilarities_AOA(const ReceiverInfo& info)
{
	if (!info.m_isValid) {
		return false;
	}

	if (info.m_multipathInfo.size() < m_multipathInfo.size()) {				//若需要加入的信息维度与当前信息维度小，则无效。
		return false;
	}
	int similarNum = 0;
	RtLbsType maxAOAOffset = 0.0;
	for (auto& curPathInfo : info.m_multipathInfo) {
		size_t hashcode = curPathInfo.GetHash();
		auto pos = pathInfoMap.find(hashcode);
		if (pos == pathInfoMap.end()) {
			continue;
		}
		auto& mainPathInfo = pos->second;
		RtLbsType curAOAPhiOffset = std::abs(mainPathInfo.m_aoAPhi - curPathInfo.m_aoAPhi);
		maxAOAOffset = std::max(maxAOAOffset, curAOAPhiOffset);									//计算最大角度偏移量
		similarNum++;
	}
	if (similarNum != static_cast<int>(m_multipathInfo.size())) {			//未达到相同多径信息数，返回false
		return false;
	}
	RtLbsType curDistance = (m_point - info.m_point).Length();
	AOA_Distance p = { maxAOAOffset, curDistance };
	m_AOADistanceInfo.push_back(p);
	return true;
}

bool ReceiverInfo::CanAddToSimilarities_TOA(const ReceiverInfo& info)
{
	if (!info.m_isValid) {
		return false;
	}

	if (info.m_multipathInfo.size() < m_multipathInfo.size()) {				//若需要加入的信息维度与当前信息维度小，则无效。
		return false;
	}
	int similarNum = 0;
	RtLbsType meanTOAOffset = 0;
	for (auto& curPathInfo : info.m_multipathInfo) {
		size_t hashcode = curPathInfo.GetHash();
		auto pos = pathInfoMap.find(hashcode);
		if (pos == pathInfoMap.end()) {
			continue;
		}
		auto& mainPathInfo = pos->second;
		RtLbsType curTOAOffset = std::abs(mainPathInfo.m_timeDelay - curPathInfo.m_timeDelay);
		meanTOAOffset = std::max(meanTOAOffset, curTOAOffset);									//计算最大角度偏移量
		similarNum++;
	}
	if (similarNum != static_cast<int>(m_multipathInfo.size())) {			//未达到相同多径信息数，返回false
		return false;
	}
	RtLbsType curDistance = (m_point - info.m_point).Length();
	TOA_Distance p = { meanTOAOffset, curDistance };
	m_TOADistanceInfo.push_back(p);
	return true;
}

bool ReceiverInfo::CanAddToSimilarities_AOATDOA(const ReceiverInfo& info)
{
	if (!info.m_isValid) {
		return false;
	}

	if (info.m_multipathInfo.size() < m_multipathInfo.size()) {				//若需要加入的信息维度与当前信息维度小，则无效。
		return false;
	}
	int similarNum = 0;
	RtLbsType minAOAOffset = FLT_MAX;
	RtLbsType minTOAOffset = FLT_MAX;
	RtLbsType minTDOAOffset = 0.0;

	for (auto& curPathInfo : info.m_multipathInfo) {
		size_t hashcode = curPathInfo.GetHash();
		auto pos = pathInfoMap.find(hashcode);
		if (pos == pathInfoMap.end()) {
			continue;
		}
		auto& mainPathInfo = pos->second;
		RtLbsType curAOAOffset = std::abs(mainPathInfo.m_aoAPhi - curPathInfo.m_aoAPhi);
		RtLbsType curTOAOffset = std::abs(mainPathInfo.m_timeDelay - curPathInfo.m_timeDelay);
		RtLbsType curTDOAOffset = std::abs(mainPathInfo.m_timeDifference - curPathInfo.m_timeDifference);
		minAOAOffset = std::min(minAOAOffset, curAOAOffset);									//计算最大角度偏移量
		minTOAOffset = std::min(minTOAOffset, curTOAOffset);									//计算最大角度偏移量
		minTDOAOffset += curTDOAOffset;															//计算最大时延差偏移量

		similarNum++;
	}
	minTDOAOffset /= similarNum;
	if (similarNum != static_cast<int>(m_multipathInfo.size())) {			//未达到相同多径信息数，返回false
		return false;
	}
	RtLbsType curDistance = (m_point - info.m_point).Length();
	AOA_Distance d1 = { minAOAOffset, curDistance };
	TOA_Distance d2 = { minTOAOffset, curDistance };
	AOA_TDOA_Distance d3 = { minAOAOffset ,minTDOAOffset/1.3, curDistance };
	m_AOADistanceInfo.push_back(d1);
	m_TOADistanceInfo.push_back(d2);
	m_AOA_TDOADistanceInfo.push_back(d3);
	return true;
}

void ReceiverInfo::Init(const RaytracingResult& rtResult)
{
	if (rtResult.m_multipathInfo.size() == 0) {
		m_isValid = false;
		return;
	}
	std::vector<PathInfo> infos = rtResult.m_multipathInfo;
	std::sort(infos.begin(), infos.end(), ComparedByPower_PathInfo);			//按照功率大小进行排序
	RtLbsType maxPower = infos.front().m_power;

	//抽取峰值后35dB后作为有效多径
	for (auto& info : infos) {
		if (info.m_rayPathType == RAYPATH_TERRAIN_REFLECTION || info.m_rayPathType == RAYPATH_TERRAIN_DIFFRACTION) {
			continue;
		}
		if ((maxPower - info.m_power) < 35) {
			m_multipathInfo.push_back(info);
		}
	}
	for (auto& info : m_multipathInfo) {
		size_t hashcode = info.GetHash();
		pathInfoMap[hashcode] = info;
	}
}

void ReceiverInfo::UpdateSimilaritiesDistance_AOA()
{
	int similarityNum = m_AOADistanceInfo.size();
	for (int i = 0; i < similarityNum; ++i) {
		m_maxDistance = std::max(m_maxDistance, m_AOADistanceInfo[i].distance);
		m_meanDistance += m_AOADistanceInfo[i].distance;
	}
	if (similarityNum == 0) {
		return;
	}
	m_meanDistance /= similarityNum;
}

void ReceiverInfo::UpdateSimilaritiesDistance_TOA()
{
	int similarityNum = m_TOADistanceInfo.size();
	if (similarityNum == 0) {
		return;
	}
	for (int i = 0; i < similarityNum; ++i) {
		m_maxDistance = std::max(m_maxDistance, m_TOADistanceInfo[i].distance);
		m_meanDistance += m_TOADistanceInfo[i].distance;
	}
	m_meanDistance /= similarityNum;
}

void ReceiverInfo::UpdateSimilaritiesDistance_AOATDOA()
{
	int similarityNum = m_AOA_TDOADistanceInfo.size();
	for (int i = 0; i < similarityNum; ++i) {
		m_maxDistance = std::max(m_maxDistance, m_AOA_TDOADistanceInfo[i].distance);
		m_meanDistance += m_AOA_TDOADistanceInfo[i].distance;
	}
	if (similarityNum == 0) {
		return;
	}
	m_meanDistance /= similarityNum;
}

void ReceiverInfo::GetDistanceByAOA(RtLbsType phi, RtLbsType& maxDistance, RtLbsType& meanDistance) const
{
	int validNum = 0;
	for (auto& p : m_AOADistanceInfo) {
		if (p.angle <= phi) {
			maxDistance = std::max(maxDistance, p.distance);
			meanDistance += p.distance;
			validNum++;
		}
	}
	if (validNum != 0) {
		meanDistance /= validNum;
	}
}

void ReceiverInfo::GetDistanceByTOA(RtLbsType delay, RtLbsType& maxDistance, RtLbsType& meanDistance) const
{
	int validNum = 0;
	for (auto& p : m_TOADistanceInfo) {
		if (p.time < delay/6) {
			maxDistance = std::max(maxDistance, p.distance);
			meanDistance += p.distance;
			validNum++;
		}
	}
	if (validNum != 0) {
		meanDistance /= validNum;
	}
}

void ReceiverInfo::GetDistanceByAOATDOA(RtLbsType phi, RtLbsType timeDiff, RtLbsType& maxDistance, RtLbsType& meanDistance) const
{
	int validNum = 0;
	for (auto& p : m_AOA_TDOADistanceInfo) {
		if (p.angle <= phi && p.timeDifference <= timeDiff) {
			maxDistance = std::max(maxDistance, p.distance);
			meanDistance += p.distance;
			validNum++;
		}
	}
	if (validNum != 0) {
		meanDistance /= validNum;
	}
}

void ReceiverInfo::Write2File(std::ofstream& stream)
{
	stream << m_point.x << "\t" << m_point.y << "\t" << m_meanDistance << std::endl;
}
