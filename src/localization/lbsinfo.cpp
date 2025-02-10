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
	else if (lbsMethod == LBS_METHOD_RT_TDOA) {																	//纯TDOA方法
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_TDOA(newSource);
			m_sources[i] = newSource;
		}
		m_originalSources = m_sources;
		EraseRepeatGeneralSources(m_sources);																	//消除掉重复的广义源
	}
	else if (lbsMethod == LBS_METHOD_RT_TOA) {
		m_sources.reserve(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			RtLbsType propagation_t = m_nodes[i]->m_sensorData->m_time * LIGHT_VELOCITY_AIR;
			if (m_nodes[i]->m_type == NODE_ROOT) {															//对于根节点直接入
				GeneralSource* newSource = new GeneralSource();
				m_nodes[i]->GetGeneralSource_AOA(newSource);
				m_sources.push_back(newSource);
			}
			else {
				if (propagation_t >= m_nodes[i]->m_tMin && propagation_t <= m_nodes[i]->m_tMax) {			//当且仅当测量到传播距离大于节点本身的距离时广义源才有效
					GeneralSource* newSource = new GeneralSource();
					m_nodes[i]->GetGeneralSource_AOA(newSource);
					m_sources.push_back(newSource);
				}
			}
		}
		m_originalSources = m_sources;
		EraseRepeatGeneralSources(m_sources);																	//消除掉重复的广义源
	}
	else if (lbsMethod == LBS_METHOD_RT_AOA_TOA) {
		m_sources.resize(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); ++i) {
			GeneralSource* newSource = new GeneralSource();
			m_nodes[i]->GetGeneralSource_AOA(newSource);
			m_sources[i] = newSource;
		}
		m_originalSources = m_sources;
		//EraseRepeatGeneralSources(m_sources);																	//消除掉重复的广义源
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
	if (lbsMethod == LBS_METHOD_RT_TOA) {												//产生TOA算法的广义源情况下需要限制广义源传播时间
		for (auto& curData : sensorDatas) {
			std::vector<GeneralSource*> tempSources;			/** @brief	存储临时的广义源	*/
			tempSources.reserve(m_nodes.size());
			RtLbsType propagation_t = curData.m_time * LIGHT_VELOCITY_AIR;
			for (int i = 0; i < m_nodes.size(); ++i) {
				if (propagation_t >= m_nodes[i]->m_tMin && propagation_t <= m_nodes[i]->m_tMax) {			//当且仅当测量到传播距离大于节点本身的距离时广义源才有效
					GeneralSource* newSource = new GeneralSource();
					m_nodes[i]->GetGeneralSource_TOA(newSource, curData);
					tempSources.push_back(newSource);
				}
			}
			m_originalSources.insert(m_originalSources.end(), tempSources.begin(), tempSources.end());
			EraseRepeatGeneralSources(tempSources);																	//消除掉重复的广义源
			//将tempSource拷贝至m_source中
			int oldSize = m_sources.size();
			m_sources.resize(oldSize + tempSources.size());
			std::copy(tempSources.begin(), tempSources.end(), m_sources.begin() + oldSize);
		}
	}
	else if (lbsMethod == LBS_METHOD_RT_TDOA) {											//产生TDOA算法的广义源不需要限制任何传播时延
		for (auto& curData : sensorDatas) {
			std::vector<GeneralSource*> tempSources;			/** @brief	存储临时的广义源	*/
			tempSources.reserve(m_nodes.size());
			RtLbsType propagation_t = curData.m_time * LIGHT_VELOCITY_AIR;
			for (int i = 0; i < m_nodes.size(); ++i) {
				GeneralSource* newSource = new GeneralSource();
				m_nodes[i]->GetGeneralSource_TOA(newSource, curData);
				tempSources.push_back(newSource);
			}
			m_originalSources.insert(m_originalSources.end(), tempSources.begin(), tempSources.end());
			EraseRepeatGeneralSources(tempSources);																	//消除掉重复的广义源
			//将tempSource拷贝至m_source中
			int oldSize = m_sources.size();
			m_sources.resize(oldSize + tempSources.size());
			std::copy(tempSources.begin(), tempSources.end(), m_sources.begin() + oldSize);
		}
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
			//删除重复项
			//delete curSource;				//修复问题，由于这里为引用内存，因此禁止在此处释放内存
		}
	}

	sources.clear();
	for (auto& data : sourceMap) {
		GeneralSource* curSource = data.second;
		sources.push_back(curSource);
	}

	//重新计算phi值
	for (auto& curSource : sources) {
		curSource->UpdateEvenPhiValue();
		curSource->m_phiRepeatCount = 1;							//角度平均完成之后需要恢复重新计数状态
	}
}