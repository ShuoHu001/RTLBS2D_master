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

	if (lbsMethod != LBS_METHOD_RT_TDOA) {			//无AOA角度时的定位模式时必须要过滤掉重复的广义源
		return;
	}

	EraseRepeatGeneralSources(m_sources);			//消除掉重复的广义源
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
			curSource->m_isValid = false;
			GeneralSource* existingSource = pos->second;
			existingSource->m_sensorData.m_phi += curSource->m_sensorData.m_phi;			//角度叠加
			if (existingSource->m_weight < curSource->m_weight) {							//针对重复的广义源，其权重保留最大值
				existingSource->m_weight = curSource->m_weight;
			}
			existingSource->m_phiRepeatCount += 1;
			if (existingSource->m_wCount < curSource->m_wCount) {				//更新计数权重
				existingSource->m_wCount = curSource->m_wCount;
			}
			//同时释放重复的广义源
			delete curSource;
			curSource = nullptr;
		}
	}

	//删除source中无效的数据
	sources.erase(std::remove_if(sources.begin(), sources.end(), [](const GeneralSource* s) {
		return s==nullptr;
		}), sources.end());

	//重新计算phi值
	for (auto it = sources.begin(); it != sources.end(); ++it) {
		(*it)->UpdateEvenPhiValue();
	}
}
