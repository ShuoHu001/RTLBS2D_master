#ifndef RTLBS_OUTPUTCONFIG
#define RTLBS_OUTPUTCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
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
const std::string KEY_OUTPUTCONFIG_OUTPUTGSFORCRLB = "OutputGSForCRLB";
const std::string KEY_OUTPUTCONFIG_OUTPUTLBSMETHOD = "OutputLBSMethod";
const std::string KEY_OUTPUTCONFIG_OUTPUTSENSORDATASPARSEFACTOR = "OutputSensorDataSparseFactor";

class OutputConfig:public Serializable { 
	
public:
	std::string m_rtDirectory;							/** @brief	输出文件目录-射线追踪算法	*/
	std::string m_lbsDirectory;							/** @brief	输出文件目录-定位算法	*/
	bool m_outputPower;									/** @brief	输出功率	*/
	bool m_outputLoss;									/** @brief	输出损耗	*/
	bool m_outputMagnitude;								/** @brief	输出电场强度	*/
	bool m_outputMultipath;								/** @brief	输出多径	*/
	bool m_outputPDP;									/** @brief	输出功率时延信息	*/
	bool m_outputCFR;									/** @brief	输出信道频率响应	*/
	bool m_outputCIR;									/** @brief	输出	信道冲激响应*/
	bool m_outputAOA;									/** @brief	输出到达角	*/
	bool m_outputAOD;									/** @brief	输出离开角	*/
	bool m_outputSensorDataSPSTMD;						/** @brief	输出单站单源多数据定位模式传感器数据	*/
	bool m_outputSensorDataMPSTSD;						/** @brief	输出多站单源单数据定位模式传感器数据	*/
	bool m_outputSensorDataSPMTMD;						/** @brief	输出单站多源多数据定位模式传感器数据	*/
	bool m_outputSensorDataMPMTMD;						/** @brief	输出多站多源多数据定位模式传感器数据	*/
	bool m_outputGSForCRLB;								/** @brief	输出广义源-计算CRLB	*/
	LOCALIZATION_METHOD m_outputLBSMethod;				/** @brief	输出定位传感器数据时候的定位模式	*/
	RtLbsType m_outputSensorDataSparseFactor;			/** @brief	输出传感器数据的稀疏程度，1为不稀疏，0为极度稀疏	*/

public:
	OutputConfig();
	OutputConfig(const OutputConfig& config);
	~OutputConfig();
	OutputConfig& operator = (const OutputConfig& config);
	bool IsValid() const;								//验证数据是否有效S
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};

#endif
