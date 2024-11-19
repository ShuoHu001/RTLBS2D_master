#ifndef RTLBS_ANTENNACONFIG
#define RTLBS_ANTENNACONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "math/vector3d.h"
#include "math/euler.h"

const std::string KEY_ANTENNACONFIG_ANTID = "AntId";
const std::string KEY_ANTENNACONFIG_TYPEID = "TypeId";
const std::string KEY_ANTENNACONFIG_ANTNAME = "AntName";
const std::string KEY_ANTENNACONFIG_TYPECALGAIN = "TypicalGain";
const std::string KEY_ANTENNACONFIG_FREQMIN = "FreqMin";
const std::string KEY_ANTENNACONFIG_FREQMAX = "FreqMax";
const std::string KEY_ANTENNACONFIG_POLARIZATION = "Polarization";
const std::string KEY_ANTENNACONFIG_POSTURE = "Posture";
const std::string KEY_ANTENNACONFIG_PATTERNFILENAME = "PatternFileName";

class AntennaConfig :public Serializable {
public:
	int m_antId;					/** @brief	天线ID	*/
	int m_typeId;					/** @brief	天线类型ID	*/
	std::string m_antName;			/** @brief	天线名称	*/
	RtLbsType m_typicalGain;		/** @brief	天线典型增益	*/
	RtLbsType m_freqMin;			/** @brief	天线工作的最小频率	*/
	RtLbsType m_freqMax;			/** @brief	天线工作的最大频率	*/
	Vector3D m_polarization;		/** @brief	天线极化方式	*/
	Euler m_posture;				/** @brief	天线姿态	*/
	std::string m_patternFileName;	/** @brief	天线方向图对应的文件名称	*/

public:
	AntennaConfig();
	AntennaConfig(const AntennaConfig& config);
	~AntennaConfig();
	AntennaConfig& operator = (const AntennaConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};


#endif
