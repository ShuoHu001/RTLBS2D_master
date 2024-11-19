#ifndef RTLBS_MATERIALCONFIG
#define RTLBS_MATERIALCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "math/complex.h"

const std::string KEY_MATERIALCONFIG_ID = "Id";
const std::string KEY_MATERIALCONFIG_NAME = "Name";
const std::string KEY_MATERIALCONFIG_PERMITTIVITY = "Permittivity";
const std::string KEY_MATERIALCONFIG_CONDUCTIVITY = "Conductivity";
const std::string KEY_MATERIALCONFIG_PENETRATIONLOSS = "PenetrationLoss";

class MaterialConfig :public Serializable {
public:
	int m_id;							/** @brief	����ID	*/
	std::string m_name;					/** @brief	��������	*/
	RtLbsType m_permittivity;			/** @brief	������Խ�糣��	*/
	RtLbsType m_conductivity;			/** @brief	���ϵ絼��	*/
	RtLbsType m_penetrationLoss;		/** @brief	�����ڲ��Ĳ�����ģ�ͳ��ֵ	*/

public:
	MaterialConfig();
	MaterialConfig(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity);
	MaterialConfig(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity, RtLbsType penetractionLoss);
	MaterialConfig(const MaterialConfig& mat);
	~MaterialConfig();
	MaterialConfig& operator = (const MaterialConfig& mat);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);			//���л�Ϊ����
	bool Deserialize(const rapidjson::Value& value);									//�����л�
};


#endif
