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
	LBSERRORTYPE m_errorType;									/** @brief	误差类型	*/
	std::vector<RtLbsType> m_phiErrorSigmas;					/** @brief	角度误差数据(单位弧度)	*/
	std::vector<RtLbsType> m_timeMErrorSigmas;					/** @brief	时间误差数据(单位m)	*/
private:
	std::vector<RtLbsType> m_phiDegreeErrorSigmas;				/** @brief	角度误差数据(单位°)	*/
	std::vector<RtLbsType> m_timeNSErrorSigmas;					/** @brief	时间误差数据(单位ns)	*/
public:
	LBSErrorConfig();
	~LBSErrorConfig();
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
