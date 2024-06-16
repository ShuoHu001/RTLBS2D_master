#include "lbsresult.h"

LBSResult::LBSResult()
	: m_pathNum(0)
	, m_freqNum(0)
	, m_sensor(nullptr)
	, m_receiver(nullptr)
	, m_featureSize(0)
	, m_powerSTDValue(0.0)
{
}

LBSResult::~LBSResult()
{
}

void LBSResult::SetRayPath(std::vector<RayPath3D*>& paths)
{
	m_paths = paths;
}

void LBSResult::CalculateBaseInfo(std::vector<RtLbsType>& freqs, MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction)
{
	//内存分配
	m_pathNum = static_cast<int>(m_paths.size());
	m_freqs = freqs;
	m_freqNum = static_cast<int>(m_freqs.size());

	m_emittedPower.resize(m_freqNum * m_pathNum);

	//计算特征数
	std::vector<int> featureId(m_pathNum);
	for (int i = 0; i < m_pathNum; ++i) {
		featureId[i] = m_paths[i]->m_angularSpectrumCategoryId;
	}

	std::sort(featureId.begin(), featureId.end());
	featureId.erase(std::unique(featureId.begin(), featureId.end()));
	m_featureSize = featureId.size();

	//计算电磁场
	for (int i = 0; i < m_freqNum; ++i) {
		for (int j = 0; j < m_pathNum; ++j) {
			int offset = i * m_pathNum + j;
			//根据输入的传感器和接收机，计算到达接收机的功率dBm
			Antenna* transmitterAntenna = m_receiver->m_antenna;
			Antenna* receiverAntenna = m_sensor->m_antenna;
			RtLbsType powerIn = 1.0;//默认设定功率为1W，也就是30dBm
			Complex vectorEFiled = m_paths[j]->CalculateStrengthFieldReverse(powerIn, m_freqs[i], tranFunction, matLibrary, transmitterAntenna, receiverAntenna);
			RtLbsType scalarEField = vectorEFiled.MValue();
			RtLbsType gain = m_sensor->GetGain() + m_receiver->GetGain();
			RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (m_freqs[i] / QUARTER_PI)) - 20 * log10(30);		//中间变量
			RtLbsType powerOut = 20 * log10(scalarEField) + conterm + 30 + gain;
			RtLbsType loss = 30 - powerOut;									/** @brief	多径损耗	*/
			//按照多径AOA值，获取其角度功率谱对应的功率，计算得到其推算发射功率
			RtLbsType aoaTheta = m_paths[j]->GetAOA_Theta();
			//m_emittedPower[offset] = m_sensor->m_angularSpectrum.GetReceivedPower(aoaTheta) + loss;					//计算从receiver出射的功率
		}
		
	}

}

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
