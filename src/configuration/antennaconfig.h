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
	int m_antId;					/** @brief	����ID	*/
	int m_typeId;					/** @brief	��������ID	*/
	std::string m_antName;			/** @brief	��������	*/
	RtLbsType m_typicalGain;		/** @brief	���ߵ�������	*/
	RtLbsType m_freqMin;			/** @brief	���߹�������СƵ��	*/
	RtLbsType m_freqMax;			/** @brief	���߹��������Ƶ��	*/
	Vector3D m_polarization;		/** @brief	���߼�����ʽ	*/
	Euler m_posture;				/** @brief	������̬	*/
	std::string m_patternFileName;	/** @brief	���߷���ͼ��Ӧ���ļ�����	*/

public:
	AntennaConfig();
	AntennaConfig(const AntennaConfig& config);
	~AntennaConfig();
	AntennaConfig& operator = (const AntennaConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};


#endif
