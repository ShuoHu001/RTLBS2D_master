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
	uint8_t m_limitTotal;									/** @brief	��������	*/
	uint8_t m_limitReflect;									/** @brief	����������	*/
	uint8_t m_limitTransmit;								/** @brief	͸��������	*/
	uint8_t m_limitDiffract;								/** @brief	����������	*/
	uint8_t m_limitScatter;									/** @brief	ɢ��������	*/
	uint8_t m_depth;										/** @brief	�������	*/

public:
	HOST_DEVICE_FUNC LimitInfo();
	HOST_DEVICE_FUNC LimitInfo(uint8_t limitTotal, uint8_t limitReflect, uint8_t limitTransmit, uint8_t limitDiffract, uint8_t limitScatter);
	HOST_DEVICE_FUNC LimitInfo(const LimitInfo& info);
	HOST_DEVICE_FUNC ~LimitInfo();
	HOST_DEVICE_FUNC LimitInfo& operator = (const LimitInfo& info);
	HOST_DEVICE_FUNC bool operator == (const LimitInfo& info) const;
	HOST_DEVICE_FUNC bool operator != (const LimitInfo& info) const;
	HOST_DEVICE_FUNC bool IsValid() const;													//��ѯ�Ƿ�����Ч��
	HOST_DEVICE_FUNC void MinusReflectLimit();												//���ٷ���������
	HOST_DEVICE_FUNC void MinusTransmitLimit(PATHNODETYPE type);							//����͸��������
	HOST_DEVICE_FUNC void MinusDiffractLimit();												//��������������
	HOST_DEVICE_FUNC void MinusScatterLimit();												//����ɢ��������
	HOST_DEVICE_FUNC void SetETranLimitInfo();												//���þ���͸�������������
	HOST_DEVICE_FUNC EndStopInfo CalEndStopInfo() const;									//����׷����ֹ��Ϣ
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;			//���л�����
	bool Deserialize(const rapidjson::Value& value);										//�����л�����
};



#endif
