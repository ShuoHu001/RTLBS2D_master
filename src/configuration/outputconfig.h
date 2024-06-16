#ifndef RTLBS_OUTPUTCONFIG
#define RTLBS_OUTPUTCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"

const std::string KEY_OUTPUTCONFIG_RTDIRECTORY = "RTDirectory";
const std::string KEY_OUTPUTCONFIG_LBSDIRECTORY = "LBSDirectory";
const std::string KEY_OUTPUTCONFIG_OUTPUTPOWER = "OutputPower";
const std::string KEY_OUTPUTCONFIG_OUTPUTLOSS = "OutputLoss";
const std::string KEY_OUTPUTCONFIG_OUTPUTMAGNITUDE = "OutputMagnitude";
const std::string KEY_OUTPUTCONFIG_OUTPUTMULTIPATH = "OutputMultipath";
const std::string KEY_OUTPUTCONFIG_OUTPUTPDP = "OutputPDP";
const std::string KEY_OUTPUTCONFIG_OUTPUTCIR = "OutputCIR";
const std::string KEY_OUTPUTCONFIG_OUTPUTCFR = "OutputCFR";
const std::string KEY_OUTPUTCONFIG_OUTPUTAOA = "OutputAOA";
const std::string KEY_OUTPUTCONFIG_OUTPUTAOD = "OutputAOD";
const std::string KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPSTMD = "OutputSensorDataSPSTMD";
const std::string KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPSTSD = "OutputSensorDataMPSTSD";
const std::string KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_SPMTMD = "OutputSensorDataSPMTMD";
const std::string KEY_OUTPUTCONFIG_OUTPUTSENSORDATA_MPMTMD = "OutputSensorDataMPMTMD";

class OutputConfig:public Serializable { 
	
public:
	std::string m_rtDirectory;							/** @brief	����ļ�Ŀ¼-����׷���㷨	*/
	std::string m_lbsDirectory;							/** @brief	����ļ�Ŀ¼-��λ�㷨	*/
	bool m_outputPower;									/** @brief	�������	*/
	bool m_outputLoss;									/** @brief	������	*/
	bool m_outputMagnitude;								/** @brief	����糡ǿ��	*/
	bool m_outputMultipath;								/** @brief	����ྶ	*/
	bool m_outputPDP;									/** @brief	�������ʱ����Ϣ	*/
	bool m_outputCFR;									/** @brief	����ŵ�Ƶ����Ӧ	*/
	bool m_outputCIR;									/** @brief	���	�ŵ��弤��Ӧ*/
	bool m_outputAOA;									/** @brief	��������	*/
	bool m_outputAOD;									/** @brief	����뿪��	*/
	bool m_outputSensorDataSPSTMD;						/** @brief	�����վ��Դ�����ݶ�λģʽ����������	*/
	bool m_outputSensorDataMPSTSD;						/** @brief	�����վ��Դ�����ݶ�λģʽ����������	*/
	bool m_outputSensorDataSPMTMD;						/** @brief	�����վ��Դ�����ݶ�λģʽ����������	*/
	bool m_outputSensorDataMPMTMD;						/** @brief	�����վ��Դ�����ݶ�λģʽ����������	*/

public:
	OutputConfig();
	OutputConfig(const OutputConfig& config);
	~OutputConfig();
	OutputConfig& operator = (const OutputConfig& config);
	bool IsValid() const;								//��֤�����Ƿ���ЧS
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};

#endif
