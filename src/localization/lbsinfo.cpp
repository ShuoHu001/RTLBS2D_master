#include "lbsinfo.h"

LBSInfo::LBSInfo()
	: m_sensor(nullptr)
{
}

LBSInfo::~LBSInfo()
{
	for (auto& node : m_nodes) {
		delete node;
		node = nullptr;
	}

	for (auto& source : m_originalSources) {
		delete source;
		source = nullptr;
	}
}

void LBSInfo::SetSensorData(const SensorData& data)
{
	for (auto& curSource : m_sources) {
		curSource->m_sensorData = data;
	}
}

void LBSInfo::SetNodes(std::vector<LBSTreeNode*>& nodes)
{
	m_nodes = nodes;
}

void LBSInfo::CalculateBaseInfo(LOCALIZATION_METHOD lbsMethod)
{
	if (lbsMethod == LBS_METHOD_RT_AOA) {
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_AOA(newSource);
			m_sources[i] = newSource;
		}
		m_originalSources = m_sources;
	}
	else if (lbsMethod == LBS_METHOD_RT_TDOA) {																	//��TDOA����
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_TDOA(newSource);
			m_sources[i] = newSource;
		}
		m_originalSources = m_sources;
		EraseRepeatGeneralSources(m_sources);																	//�������ظ��Ĺ���Դ
	}
	else if (lbsMethod == LBS_METHOD_RT_TOA) {
		m_sources.reserve(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			RtLbsType propagation_t = m_nodes[i]->m_sensorData->m_time * LIGHT_VELOCITY_AIR;
			if (m_nodes[i]->m_type == NODE_ROOT) {															//���ڸ��ڵ�ֱ����
				GeneralSource* newSource = new GeneralSource();
				m_nodes[i]->GetGeneralSource_AOA(newSource);
				m_sources.push_back(newSource);
			}
			else {
				if (propagation_t >= m_nodes[i]->m_tMin && propagation_t <= m_nodes[i]->m_tMax) {			//���ҽ�������������������ڽڵ㱾��ľ���ʱ����Դ����Ч
					GeneralSource* newSource = new GeneralSource();
					m_nodes[i]->GetGeneralSource_AOA(newSource);
					m_sources.push_back(newSource);
				}
			}
		}
		m_originalSources = m_sources;
		EraseRepeatGeneralSources(m_sources);																	//�������ظ��Ĺ���Դ
	}
	else if (lbsMethod == LBS_METHOD_RT_AOA_TOA) {
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_AOA(newSource);
			m_sources[i] = newSource;
		}
		m_originalSources = m_sources;
		//EraseRepeatGeneralSources(m_sources);																	//�������ظ��Ĺ���Դ
	}
	else if (lbsMethod == LBS_METHOD_RT_AOA_TDOA) {
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_AOA(newSource);
			m_sources[i] = newSource;
		}
		m_originalSources = m_sources;
	}


}

void LBSInfo::CalculateBaseInfo(std::vector<SensorData>& sensorDatas, LOCALIZATION_METHOD lbsMethod)
{
	if (lbsMethod == LBS_METHOD_RT_TOA) {												//����TOA�㷨�Ĺ���Դ�������Ҫ���ƹ���Դ����ʱ��
		for (auto& curData : sensorDatas) {
			std::vector<GeneralSource*> tempSources;			/** @brief	�洢��ʱ�Ĺ���Դ	*/
			tempSources.reserve(m_nodes.size());
			RtLbsType propagation_t = curData.m_time * LIGHT_VELOCITY_AIR;
			for (int i = 0; i < m_nodes.size(); ++i) {
				if (propagation_t >= m_nodes[i]->m_tMin && propagation_t <= m_nodes[i]->m_tMax) {			//���ҽ�������������������ڽڵ㱾��ľ���ʱ����Դ����Ч
					GeneralSource* newSource = new GeneralSource();
					m_nodes[i]->GetGeneralSource_TOA(newSource, curData);
					tempSources.push_back(newSource);
				}
			}
			m_originalSources.insert(m_originalSources.end(), tempSources.begin(), tempSources.end());
			EraseRepeatGeneralSources(tempSources);																	//�������ظ��Ĺ���Դ
			//��tempSource������m_source��
			int oldSize = m_sources.size();
			m_sources.resize(oldSize + tempSources.size());
			std::copy(tempSources.begin(), tempSources.end(), m_sources.begin() + oldSize);
		}
	}
	else if (lbsMethod == LBS_METHOD_RT_TDOA) {											//����TDOA�㷨�Ĺ���Դ����Ҫ�����κδ���ʱ��
		for (auto& curData : sensorDatas) {
			std::vector<GeneralSource*> tempSources;			/** @brief	�洢��ʱ�Ĺ���Դ	*/
			tempSources.reserve(m_nodes.size());
			RtLbsType propagation_t = curData.m_time * LIGHT_VELOCITY_AIR;
			for (int i = 0; i < m_nodes.size(); ++i) {
				GeneralSource* newSource = new GeneralSource();
				m_nodes[i]->GetGeneralSource_TOA(newSource, curData);
				tempSources.push_back(newSource);
			}
			m_originalSources.insert(m_originalSources.end(), tempSources.begin(), tempSources.end());
			EraseRepeatGeneralSources(tempSources);																	//�������ظ��Ĺ���Դ
			//��tempSource������m_source��
			int oldSize = m_sources.size();
			m_sources.resize(oldSize + tempSources.size());
			std::copy(tempSources.begin(), tempSources.end(), m_sources.begin() + oldSize);
		}
	}
	
}

inline void EraseRepeatGeneralSources(std::vector<GeneralSource*>& sources)
{
	//������ע��--�Թ���Դ���г�ֵ����
	/*---------------------------------------------------------------------------------------------------------------/
	//1-����hashӳ����˵�����������Ĺ���Դ
	/---------------------------------------------------------------------------------------------------------------*/
	std::unordered_map<size_t, GeneralSource*> sourceMap;			/** @brief	hashӳ��Դͼ	*/
	for (auto it = sources.begin(); it != sources.end(); ++it) {
		GeneralSource*& curSource = *it;
		size_t hashCode = curSource->GetHash();
		auto pos = sourceMap.find(hashCode);
		if (pos == sourceMap.end()) {		//δ�ҵ��ظ��������map��
			sourceMap[hashCode] = curSource;
		}
		else {
			GeneralSource*& existingSource = pos->second;									/** @brief	���ڵ���Чָ��	*/
			existingSource->m_sensorData.m_phi += curSource->m_sensorData.m_phi;			//�Ƕȵ���
			existingSource->m_weight = std::max(existingSource->m_weight, curSource->m_weight);		//����ظ��Ĺ���Դ����Ȩ�ر������ֵ
			existingSource->m_phiRepeatCount += 1;
			//ɾ���ظ���
			//delete curSource;				//�޸����⣬��������Ϊ�����ڴ棬��˽�ֹ�ڴ˴��ͷ��ڴ�
		}
	}

	sources.clear();
	for (auto& data : sourceMap) {
		GeneralSource* curSource = data.second;
		sources.push_back(curSource);
	}

	//���¼���phiֵ
	for (auto& curSource : sources) {
		curSource->UpdateEvenPhiValue();
		curSource->m_phiRepeatCount = 1;							//�Ƕ�ƽ�����֮����Ҫ�ָ����¼���״̬
	}
}