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
	//�ڴ����
	m_pathNum = static_cast<int>(m_paths.size());
	m_freqs = freqs;
	m_freqNum = static_cast<int>(m_freqs.size());

	m_emittedPower.resize(m_freqNum * m_pathNum);

	//����������
	std::vector<int> featureId(m_pathNum);
	for (int i = 0; i < m_pathNum; ++i) {
		featureId[i] = m_paths[i]->m_angularSpectrumCategoryId;
	}

	std::sort(featureId.begin(), featureId.end());
	featureId.erase(std::unique(featureId.begin(), featureId.end()));
	m_featureSize = featureId.size();

	//�����ų�
	for (int i = 0; i < m_freqNum; ++i) {
		for (int j = 0; j < m_pathNum; ++j) {
			int offset = i * m_pathNum + j;
			//��������Ĵ������ͽ��ջ������㵽����ջ��Ĺ���dBm
			Antenna* transmitterAntenna = m_receiver->m_antenna;
			Antenna* receiverAntenna = m_sensor->m_antenna;
			RtLbsType powerIn = 1.0;//Ĭ���趨����Ϊ1W��Ҳ����30dBm
			Complex vectorEFiled = m_paths[j]->CalculateStrengthFieldReverse(powerIn, m_freqs[i], tranFunction, matLibrary, transmitterAntenna, receiverAntenna);
			RtLbsType scalarEField = vectorEFiled.MValue();
			RtLbsType gain = m_sensor->GetGain() + m_receiver->GetGain();
			RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (m_freqs[i] / QUARTER_PI)) - 20 * log10(30);		//�м����
			RtLbsType powerOut = 20 * log10(scalarEField) + conterm + 30 + gain;
			RtLbsType loss = 30 - powerOut;									/** @brief	�ྶ���	*/
			//���նྶAOAֵ����ȡ��Ƕȹ����׶�Ӧ�Ĺ��ʣ�����õ������㷢�书��
			RtLbsType aoaTheta = m_paths[j]->GetAOA_Theta();
			//m_emittedPower[offset] = m_sensor->m_angularSpectrum.GetReceivedPower(aoaTheta) + loss;					//�����receiver����Ĺ���
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
