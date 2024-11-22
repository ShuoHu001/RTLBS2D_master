#ifndef RTLBS_SOLVINGCONFIG_H
#define RTLBS_SOLVINGCONFIG_H

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"

const std::string KEY_SOLVINGCONFIG_SOLVINGMODE = "SolvingMode";
const std::string KEY_SOLVINGCONFIG_LOSSTYPE = "LossType";
const std::string KEY_SOLVINGCONFIG_ITERNUM = "IterNum";
const std::string KEY_SOLVINGCONFIG_TOLERANCE = "Tolerance";

class SolvingConfig {
public:
    SOLVINGMODE m_solvingMode;
    LOSSFUNCTIONTYPE m_lossType;
    int m_iterNum;
    RtLbsType m_tolerance;
public:
    SolvingConfig();
    SolvingConfig(const SolvingConfig& config);
    ~SolvingConfig();
    SolvingConfig& operator = (const SolvingConfig& config);
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
