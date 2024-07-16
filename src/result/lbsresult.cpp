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
		//更新source的父节点
		for (auto& curSource : m_sources) {
			int fatherSourceId = curSource->m_originPathNode.m_fatherNodeId;
			GeneralSource* fatherSource = nullptr;
			if (fatherSourceId != -1) {
				fatherSource = m_sources[fatherSourceId];
			}
			curSource->m_fatherSource = fatherSource;
		}
	}
	else if (lbsMethod == LBS_METHOD_RT_TDOA) {																	//纯TDOA方法
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_TDOA(newSource);
			m_sources[i] = newSource;
		}
		//更新source的父节点
		for (auto& curSource : m_sources) {
			int fatherSourceId = curSource->m_originPathNode.m_fatherNodeId;
			GeneralSource* fatherSource = nullptr;
			if (fatherSourceId != -1) {
				fatherSource = m_sources[fatherSourceId];
			}
			curSource->m_fatherSource = fatherSource;
		}
		EraseRepeatGeneralSources(m_sources);																	//消除掉重复的广义源
	}

	
}

inline void EraseRepeatGeneralSources(std::vector<GeneralSource*>& sources)
{
	//功能项注释--对广义源进行初值过滤
	/*---------------------------------------------------------------------------------------------------------------/
	//1-采用hash映射过滤掉反射中冗余的广义源
	/---------------------------------------------------------------------------------------------------------------*/
	std::unordered_map<size_t, GeneralSource*> sourceMap;			/** @brief	hash映射源图	*/
	for (auto it = sources.begin(); it != sources.end(); ++it) {
		GeneralSource*& curSource = *it;
		size_t hashCode = curSource->GetHash();
		auto pos = sourceMap.find(hashCode);
		if (pos == sourceMap.end()) {		//未找到重复项，加入至map中
			sourceMap[hashCode] = curSource;
		}
		else {
			GeneralSource*& existingSource = pos->second;									/** @brief	存在的有效指针	*/
			existingSource->m_sensorData.m_phi += curSource->m_sensorData.m_phi;			//角度叠加
			existingSource->m_weight = std::max(existingSource->m_weight, curSource->m_weight);		//针对重复的广义源，其权重保留最大值
			existingSource->m_phiRepeatCount += 1;
		}
	}

	sources.clear();
	for (auto& data : sourceMap) {
		GeneralSource* curSource = data.second;
		sources.push_back(curSource);
	}


	//重新计算phi值
	for (auto it = sources.begin(); it != sources.end(); ++it) {
		(*it)->UpdateEvenPhiValue();
	}
}
