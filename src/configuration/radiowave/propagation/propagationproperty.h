#ifndef RTLBS_PROPAGATIONPROPERTY
#define RTLBS_PROPAGATIONPROPERTY

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"

const std::string KEY_PROPAGATIONPROPERTY_HASREFLECT = "HasReflect";
const std::string KEY_PROPAGATIONPROPERTY_HASTRANSMIT = "HasTransmit";
const std::string KEY_PROPAGATIONPROPERTY_HASEMPIRICALTRANSMIT = "HasEmpiricalTransmit";
const std::string KEY_PROPAGATIONPROPERTY_HASDIFFRACT = "HasDiffract";
const std::string KEY_PROPAGATIONPROPERTY_HASSCATTERING = "HasScattering";
const std::string KEY_PROPAGATIONPROPERTY_TERRAINDIFFRACTIONMODE = "TerrainDiffractionMode";

class PropagationProperty {
public:
	bool m_hasRelfection;										/** @brief	�Ƿ��з���	*/
	bool m_hasDiffraction;										/** @brief	�Ƿ�������	*/
	bool m_hasTransmission;										/** @brief	�Ƿ���͸�䣨����������͸�䣩	*/
	bool m_hasEmpiricalTransmission;							/** @brief	�Ƿ��о���͸��	*/
	bool m_hasScattering;										/** @brief	�Ƿ���ɢ��	*/
	TERRAINDIFFRACTIONMODE m_terrainDiffractionMode;			/** @brief	��������ģʽ	*/

public:
	HOST_DEVICE_FUNC PropagationProperty();
	HOST_DEVICE_FUNC PropagationProperty(const PropagationProperty& property);
	HOST_DEVICE_FUNC PropagationProperty(bool hasReflection, bool hasDiffraction, bool hasTransmission, bool hasEmpiricalTransmission, bool hasScattering);
	HOST_DEVICE_FUNC ~PropagationProperty();
	HOST_DEVICE_FUNC PropagationProperty& operator = (const PropagationProperty& property);
	HOST_DEVICE_FUNC bool operator == (const PropagationProperty& property) const;
	HOST_DEVICE_FUNC bool operator != (const PropagationProperty& property) const;
	HOST_DEVICE_FUNC void Set(bool hasReflection, bool hasDiffraction, bool hasTransmission, bool hasEmpiricalTransmission, bool hasScattering);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
