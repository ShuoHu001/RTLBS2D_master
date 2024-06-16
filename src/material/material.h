#ifndef RTLBS_MATERIAL
#define RTLBS_MATERIAL

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "math/complex.h"

const std::string KEY_MATERIAL_ID = "Id";
const std::string KEY_MATERIAL_NAME = "Name";
const std::string KEY_MATERIAL_PERMITTIVITY = "Permittivity";
const std::string KEY_MATERIAL_CONDUCTIVITY = "Conductivity";
const std::string KEY_MATERIAL_PENETRATIONLOSS = "PenetrationLoss";

class Material:public Serializable {
public:
	int m_id;							/** @brief	材料ID	*/
	std::string m_name;					/** @brief	材料名称	*/
	RtLbsType m_permittivity;			/** @brief	材料相对介电常数	*/
	RtLbsType m_conductivity;			/** @brief	材料电导率	*/
	RtLbsType m_penetrationLoss;		/** @brief	材料内部的步进损耗，统计值	*/
private:
	RtLbsType m_refractiveN;			/** @brief	介质折射率	*/

public:
	Material();
	Material(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity);
	Material(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity, RtLbsType penetractionLoss);
	Material(const Material& mat);
	~Material();
	Material& operator = (const Material& mat);
	bool operator == (const Material& mat) const;
	bool operator != (const Material& mat) const;
	Complex GetComplexForm(RtLbsType freq) const;			//获得材质的复数形式
	RtLbsType GetRefractiveN() const;						//获得材质的折射率

	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);			//序列化为对象
	bool Deserialize(const rapidjson::Value& value);									//反序列化
};

#endif
