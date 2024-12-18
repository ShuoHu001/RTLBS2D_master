#ifndef RTLBS_LBSERRORCONFIG_H
#define RTLBS_LBSERRORCONFIG_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"

const std::string KEY_LBSERRORCONFIG_ERRORTYPE = "ErrorType";
const std::string KEY_LBSERRORCONFIG_PHIDEGREEERRORSIGMAS = "PhiDegreeErrorSigmas";
const std::string KEY_LBSERRORCONFIG_TIMENSERRORSIGMAS = "TimeNSErrorSigmas";

class LBSErrorConfig {
public:
	LBSERRORTYPE m_errorType;									/** @brief	�������	*/
	std::vector<RtLbsType> m_phiErrorSigmas;					/** @brief	�Ƕ��������(��λ����)	*/
	std::vector<RtLbsType> m_timeMErrorSigmas;					/** @brief	ʱ���������(��λm)	*/
private:
	std::vector<RtLbsType> m_phiDegreeErrorSigmas;				/** @brief	�Ƕ��������(��λ��)	*/
	std::vector<RtLbsType> m_timeNSErrorSigmas;					/** @brief	ʱ���������(��λns)	*/
public:
	LBSErrorConfig();
	~LBSErrorConfig();
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
