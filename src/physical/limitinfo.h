#ifndef RTLBS_LIMITINFO
#define RTLBS_LIMITINFO

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/struct.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"

const std::string KEY_LIMITINFO = "LimitInfo";
const std::string KEY_LIMITINFO_LIMITTOTAL = "LimitTotal";
const std::string KEY_LIMITINFO_LIMITREFLECT = "LimitReflect";
const std::string KEY_LIMITINFO_LIMITTRANSMIT = "LimitTransmit";
const std::string KEY_LIMITINFO_LIMITDIFFRACT = "LimitDiffract";
const std::string KEY_LIMITINFO_LIMITSCATTER = "LimitScatter";

class LimitInfo {
public:
	uint8_t m_limitTotal;									/** @brief	总限制数	*/
	uint8_t m_limitReflect;									/** @brief	反射限制数	*/
	uint8_t m_limitTransmit;								/** @brief	透射限制数	*/
	uint8_t m_limitDiffract;								/** @brief	绕射限制数	*/
	uint8_t m_limitScatter;									/** @brief	散射限制数	*/
	uint8_t m_depth;										/** @brief	限制深度	*/

public:
	HOST_DEVICE_FUNC LimitInfo();
	HOST_DEVICE_FUNC LimitInfo(uint8_t limitTotal, uint8_t limitReflect, uint8_t limitTransmit, uint8_t limitDiffract, uint8_t limitScatter);
	HOST_DEVICE_FUNC LimitInfo(const LimitInfo& info);
	HOST_DEVICE_FUNC ~LimitInfo();
	HOST_DEVICE_FUNC LimitInfo& operator = (const LimitInfo& info);
	HOST_DEVICE_FUNC bool operator == (const LimitInfo& info) const;
	HOST_DEVICE_FUNC bool operator != (const LimitInfo& info) const;
	HOST_DEVICE_FUNC bool IsValid() const;													//查询是否是有效的
	HOST_DEVICE_FUNC void MinusReflectLimit();												//减少反射限制数
	HOST_DEVICE_FUNC void MinusTransmitLimit(PATHNODETYPE type);							//减少透射限制数
	HOST_DEVICE_FUNC void MinusDiffractLimit();												//减少绕射限制数
	HOST_DEVICE_FUNC void MinusScatterLimit();												//减少散射限制数
	HOST_DEVICE_FUNC void SetETranLimitInfo();												//设置经验透射后限制数属性
	HOST_DEVICE_FUNC EndStopInfo CalEndStopInfo() const;									//计算追踪终止信息
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;			//序列化对象
	bool Deserialize(const rapidjson::Value& value);										//反序列化对象
};



#endif
