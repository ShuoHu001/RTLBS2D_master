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
	m_sources.resize(m_nodes.size());
	for (int i = 0; i < m_nodes.size(); ++i) {
		GeneralSource* newSource = new GeneralSource();
		m_nodes[i]->GetGeneralSource(newSource);
		m_sources[i] = newSource;
	}

	if (lbsMethod != LBS_METHOD_RT_TDOA) {			//��AOA�Ƕ�ʱ�Ķ�λģʽʱ����Ҫ���˵��ظ��Ĺ���Դ
		return;
	}

	EraseRepeatGeneralSources(m_sources);			//�������ظ��Ĺ���Դ
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
			curSource->m_isValid = false;
			GeneralSource* existingSource = pos->second;
			existingSource->m_sensorData.m_phi += curSource->m_sensorData.m_phi;			//�Ƕȵ���
			if (existingSource->m_weight < curSource->m_weight) {							//����ظ��Ĺ���Դ����Ȩ�ر������ֵ
				existingSource->m_weight = curSource->m_weight;
			}
			existingSource->m_phiRepeatCount += 1;
			if (existingSource->m_wCount < curSource->m_wCount) {				//���¼���Ȩ��
				existingSource->m_wCount = curSource->m_wCount;
			}
			//ͬʱ�ͷ��ظ��Ĺ���Դ
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
