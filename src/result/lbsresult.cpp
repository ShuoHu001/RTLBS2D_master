#include "lbsresult.h"

LBSResultGS::LBSResultGS()
	: m_sensor(nullptr)
{
}

LBSResultGS::~LBSResultGS()
{
	for (auto& node : m_nodes) {
		delete node;
		node = nullptr;
	}
	m_nodes.clear();
	std::vector<LBSTreeNode*>().swap(m_nodes);

	//for (auto& source : m_sources) {
	//	if (source != nullptr) {
	//		delete source;
	//		source = nullptr;
	//	}
	//}
	m_sources.clear();
	std::vector<GeneralSource*>().swap(m_sources);
}

void LBSResultGS::SetSensorData(const SensorData& data)
{
	for (auto& curSource : m_sources) {
		curSource->m_sensorData = data;
	}
}

void LBSResultGS::SetNodes(std::vector<LBSTreeNode*>& nodes)
{
	m_nodes = nodes;
}

void LBSResultGS::CalculateBaseInfo(LOCALIZATION_METHOD lbsMethod)
{
	if (lbsMethod == LBS_METHOD_RT_AOA || lbsMethod == LBS_METHOD_RT_AOA_TDOA) {
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_AOA(newSource);
			m_sources[i] = newSource;
		}
		//����source�ĸ��ڵ�
		for (auto& curSource : m_sources) {
			int fatherSourceId = curSource->m_originPathNode.m_fatherNodeId;
			GeneralSource* fatherSource = nullptr;
			if (fatherSourceId != -1) {
				fatherSource = m_sources[fatherSourceId];
			}
			curSource->m_fatherSource = fatherSource;
		}
	}
	else if (lbsMethod == LBS_METHOD_RT_TDOA) {																	//��TDOA����
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_TDOA(newSource);
			m_sources[i] = newSource;
		}
		//����source�ĸ��ڵ�
		for (auto& curSource : m_sources) {
			int fatherSourceId = curSource->m_originPathNode.m_fatherNodeId;
			GeneralSource* fatherSource = nullptr;
			if (fatherSourceId != -1) {
				fatherSource = m_sources[fatherSourceId];
			}
			curSource->m_fatherSource = fatherSource;
		}
		EraseRepeatGeneralSources(m_sources);																	//�������ظ��Ĺ���Դ
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
		}
	}

	sources.clear();
	for (auto& data : sourceMap) {
		GeneralSource* curSource = data.second;
		sources.push_back(curSource);
	}


	//���¼���phiֵ
	for (auto it = sources.begin(); it != sources.end(); ++it) {
		(*it)->UpdateEvenPhiValue();
	}
}
