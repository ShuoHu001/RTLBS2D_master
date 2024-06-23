#include "lbsresult.h"

LBSResultGS::LBSResultGS()
	: m_sensor(nullptr)
{
}

LBSResultGS::~LBSResultGS()
{
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
			if (curSource->m_fatherSource != nullptr && curSource->m_fatherSource->m_isValid == false) {
				curSource->m_fatherSource = curSource->m_fatherSource->m_replaceValidSource;
			}
			sourceMap[hashCode] = curSource;
		}
		else {
			//����ǰ����Դ���ڵ�Ϊ��Ч�ڵ㣬�����ƽ��ָ���滻
			GeneralSource*& existingSource = pos->second;									/** @brief	���ڵ���Чָ��	*/
			
			
			curSource->m_isValid = false;
			curSource->m_replaceValidSource = existingSource;								//����ƽ��ָ��
			existingSource->m_sensorData.m_phi += curSource->m_sensorData.m_phi;			//�Ƕȵ���
			if (existingSource->m_weight < curSource->m_weight) {							//����ظ��Ĺ���Դ����Ȩ�ر������ֵ
				existingSource->m_weight = curSource->m_weight;
			}
			existingSource->m_phiRepeatCount += 1;
			if (existingSource->m_wCount < curSource->m_wCount) {				//���¼���Ȩ��
				existingSource->m_wCount = curSource->m_wCount;
			}
		}
	}

	//ɾ��source�е���Ч����Դ(�ظ�����Ч)
	for (auto& curSource : sources) {
		if (!curSource->m_isValid) {
			delete curSource;
			curSource = nullptr;
		}
	}


	//ɾ��source����Ч������
	sources.erase(std::remove_if(sources.begin(), sources.end(), [](const GeneralSource* s) {
		return s==nullptr;
		}), sources.end());

	//���¼���phiֵ
	for (auto it = sources.begin(); it != sources.end(); ++it) {
		(*it)->UpdateEvenPhiValue();
	}
}
