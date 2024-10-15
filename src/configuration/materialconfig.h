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
	int m_id;							/** @brief	材料ID	*/
	std::string m_name;					/** @brief	材料名称	*/
	RtLbsType m_permittivity;			/** @brief	材料相对介电常数	*/
	RtLbsType m_conductivity;			/** @brief	材料电导率	*/
	RtLbsType m_penetrationLoss;		/** @brief	材料内部的步进损耗，统计值	*/

public:
	MaterialConfig();
	MaterialConfig(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity);
	MaterialConfig(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity, RtLbsType penetractionLoss);
	MaterialConfig(const MaterialConfig& mat);
	~MaterialConfig();
	MaterialConfig& operator = (const MaterialConfig& mat);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);			//序列化为对象
	bool Deserialize(const rapidjson::Value& value);									//反序列化
};


#endif
