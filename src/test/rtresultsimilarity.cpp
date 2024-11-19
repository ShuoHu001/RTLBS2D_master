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
	std::sort(infos.begin(), infos.end(), ComparedByPower_PathInfo);			//���չ��ʴ�С��������
	RtLbsType maxPower = infos.front().m_power;

	//��ȡ��ֵ��35dB����Ϊ��Ч�ྶ
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

	if (info->m_multipathInfo.size() < m_multipathInfo.size()) {				//����Ҫ�������Ϣά���뵱ǰ��Ϣά��С������Ч��
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
		maxAOAOffset = std::max(maxAOAOffset, curAOAPhiOffset);									//�������Ƕ�ƫ����
		similarNum++;
	}
	if (similarNum != static_cast<int>(m_multipathInfo.size())) {			//δ�ﵽ��ͬ�ྶ��Ϣ��������false
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
	std::sort(infos.begin(), infos.end(), ComparedByPower_PathInfo);			//���չ��ʴ�С��������
	RtLbsType maxPower = infos.front().m_power;

	//��ȡ��ֵ��35dB����Ϊ��Ч�ྶ
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
