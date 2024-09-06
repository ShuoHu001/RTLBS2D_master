#ifndef RTLBS_SENSORDATA
#define RTLBS_SENSORDATA

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "managers/randomanager.h"
#include "geometry/vector2d.h"


const std::string KEY_SENSORDATA_TIME = "Time";
const std::string KEY_SENSORDATA_TIMEDIFF = "TimeDiff";
const std::string KEY_SENSORDATA_PHI_DEGREE = "PhiDegree";
const std::string KEY_SENSORDATA_POWER = "Power";

class SensorData {
public:
	int m_id;								/** @brief	传感器数据编号,全局唯一	*/
	int m_sensorId;							/** @brief	传感器ID	*/
	RtLbsType m_time;						/** @brief	传感器接收到的时间, 单位 ns	*/
	RtLbsType m_timeDiff;					/** @brief	传感器接收到的时间差，单位 ns	*/
	RtLbsType m_phi;						/** @brief	传感器接收到的方位角,单位 弧度	*/
	RtLbsType m_power;						/** @brief	传感器接收到的功率，单位 dBm	*/
	RtLbsType m_powerLin;					/** @brief	传感器接收到的线性功率，单位dBm	*/
	RtLbsType m_phiDegree;					/** @brief	传感器接收到的方位角 单位 °	*/

public:
	SensorData();
	SensorData(const SensorData& data);
	~SensorData();
	SensorData operator = (const SensorData& data);
	bool operator < (const SensorData& data);									//重载小于符号，按照功率大小进行逆向排序
	RtLbsType DistanceAOA2D(const SensorData& data) const;						//二维角度距离
	RtLbsType DistanceDelay(const SensorData& data) const;						//时延
	Vector2D GetDirection() const;												//将接收到的角度转换为方向矢量
	void AddSimulationError(RtLbsType phiErrorSigma, RtLbsType timeErrorSigma, RtLbsType powerErrorSigma);			//增加仿真误差
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	
};

#endif
