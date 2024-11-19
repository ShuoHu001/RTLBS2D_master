#ifndef RTLBS_SCATTERCONFIG
#define RTLBS_SCATTERCONFIG

#include "rtlbs.h"
#include "math/point3d.h"
#include "utility/serializable.h"
#include "receiverunitconfig.h"

const std::string KEY_SCATTERCONFIG_FILENAME = "FileName";
const std::string KEY_SCATTERCONFIG_POSITIONS = "Positions";
const std::string KEY_SCATTERCONFIG_VELOCITIES = "Velocities";

class ScatterConfig : public Serializable {
public:
	std::string m_fileName;									/** @brief	从文件中读取数据	*/
	std::vector<Point3D> m_positions;					/** @brief	离散点接收坐标	*/
	std::vector<RtLbsType> m_velocities;				/** @brief	离散点坐标对应的速度	*/
public:
	ScatterConfig();
	ScatterConfig(const ScatterConfig& config);
	~ScatterConfig();
	ScatterConfig& operator = (const ScatterConfig& config);
	void CalculateRxPosition(std::vector<ReceiverUnitConfig>& configs);

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

#endif
