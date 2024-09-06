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
		if ((maxPower - info.m_power) < 35) {
			m_multipathInfo.push_back(info);
		}
	}
	for (auto& info : m_multipathInfo) {
		size_t hashcode = info.GetHash();
		pathInfoMap[hashcode] = &info;
	}
}

ReceiverInfo::~ReceiverInfo()
{
}

bool ReceiverInfo::CanAddToSimilarities(ReceiverInfo* info)
{
	if (!info->m_isValid) {
		return false;
	}

	if (info->m_multipathInfo.size() < m_multipathInfo.size()) {				//若需要加入的信息维度与当前信息维度小，则无效。
		return false;
	}
	int similarNum = 0;
	RtLbsType maxAOAOffset = 0.0;
	for (auto& curPathInfo : info->m_multipathInfo) {
		size_t hashcode = curPathInfo.GetHash();
		auto pos = pathInfoMap.find(hashcode);
		if (pos == pathInfoMap.end()) {
			continue;
		}
		auto& mainPathInfo = pos->second;
		RtLbsType curAOAPhiOffset = std::abs(mainPathInfo->m_aoAPhi - curPathInfo.m_aoAPhi);
		maxAOAOffset = std::max(maxAOAOffset, curAOAPhiOffset);									//计算最大角度偏移量
		similarNum++;
	}
	if (similarNum != static_cast<int>(m_multipathInfo.size())) {			//未达到相同多径信息数，返回false
		return false;
	}
	RtLbsType curDistance = (m_point - info->m_point).Length();
	auto p = std::make_pair(maxAOAOffset, curDistance);
	m_phiDistanceInfo.push_back(p);
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
		pathInfoMap[hashcode] = &info;
	}
}

void ReceiverInfo::UpdateSimilaritiesDistance()
{
	int similarityNum = m_phiDistanceInfo.size();
	for (int i = 0; i < similarityNum; ++i) {
		m_maxDistance = std::max(m_maxDistance, m_phiDistanceInfo[i].second);
		m_meanDistance += m_phiDistanceInfo[i].second;
	}
	if (similarityNum == 0) {
		return;
	}
	m_meanDistance /= similarityNum;
}

void ReceiverInfo::GetDistanceByPhi(RtLbsType phi, RtLbsType& maxDistance, RtLbsType& meanDistance) const
{
	int validNum = 0;
	for (auto& p : m_phiDistanceInfo) {
		if (p.first <= phi) {
			maxDistance = std::max(maxDistance, p.second);
			meanDistance += p.second;
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
