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
	if (rtResult.m_multipathInfo.size() == 0) {
		m_isValid = false;
		return;
	}
	m_point = rtResult.m_receiver->GetPosition2D();
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
	for (auto& curPathInfo : info->m_multipathInfo) {
		size_t hashcode = curPathInfo.GetHash();
		if (pathInfoMap.find(hashcode) == pathInfoMap.end()) {
			continue;
		}
		similarNum++;
	}
	if (similarNum != static_cast<int>(m_multipathInfo.size())) {			//未达到相同多径信息数，返回false
		return false;
	}
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
	int similarityNum = m_similarities.size();
	for (int i = 0; i < similarityNum; ++i) {
		RtLbsType curDistance = (m_point - m_similarities[i]->m_point).Length();
		m_maxDistance = std::max(m_maxDistance, curDistance);
		m_meanDistance += curDistance;
	}
	m_meanDistance /= similarityNum;
}

void ReceiverInfo::Write2File(std::ofstream& stream)
{
	stream << m_point.x << "\t" << m_point.y << "\t" << m_maxDistance << "\t" << m_meanDistance << "\t" << m_similarities.size() << std::endl;
}
