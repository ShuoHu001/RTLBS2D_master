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
			if (curSource->m_fatherSource != nullptr && curSource->m_fatherSource->m_isValid == false) {
				curSource->m_fatherSource = curSource->m_fatherSource->m_replaceValidSource;
			}
			sourceMap[hashCode] = curSource;
		}
		else {
			//若当前广义源父节点为无效节点，则进行平替指针替换
			GeneralSource*& existingSource = pos->second;									/** @brief	存在的有效指针	*/
			
			
			curSource->m_isValid = false;
			curSource->m_replaceValidSource = existingSource;								//设置平替指针
			existingSource->m_sensorData.m_phi += curSource->m_sensorData.m_phi;			//角度叠加
			if (existingSource->m_weight < curSource->m_weight) {							//针对重复的广义源，其权重保留最大值
				existingSource->m_weight = curSource->m_weight;
			}
			existingSource->m_phiRepeatCount += 1;
			if (existingSource->m_wCount < curSource->m_wCount) {				//更新计数权重
				existingSource->m_wCount = curSource->m_wCount;
			}
		}
	}

	//删除source中的无效广义源(重复性无效)
	for (auto& curSource : sources) {
		if (!curSource->m_isValid) {
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
