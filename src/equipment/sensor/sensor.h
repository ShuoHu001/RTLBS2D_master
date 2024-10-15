#ifndef RTLBS_SENSOR
#define RTLBS_SENSOR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "math/point2d.h"
#include "math/point3d.h"
#include "equipment/antenna/antenna.h"
#include "equipment/antenna/antennalibrary.h"
#include "configuration/sensor/sensorconfig.h"
#include "configuration/sensor/sensorcollectionconfig.h"
#include "sensordata.h"
#include "sensordatacollection.h"

class Sensor {
public:
	bool m_isValid;									/** @brief	传感器是否有效	*/
	int m_id;										/** @brief	传感器ID	*/
	Point3D m_position;								/** @brief	传感器位置	*/
	Antenna* m_antenna;								/** @brief	传感器天线	*/
	RtLbsType m_interLoss;							/** @brief	传感器插入损耗	*/
	RtLbsType m_attachGain;							/** @brief	传感器附加增益	*/
	RtLbsType m_phiErrorSTD;						/** @brief	传感器角度测量的标准差 单位 弧度	*/
	RtLbsType m_timeErrorSTD;						/** @brief	传感器时延测量的标准差 单位 ns	*/
	RtLbsType m_powerErrorSTD;						/** @brief	功率测量的标准差 单位 dB	*/
	SensorDataCollection m_sensorDataCollection;	/** @brief	传感器测量到的数据集合	*/

public:
	Sensor();
	Sensor(const Sensor& sensor);
	Sensor(const SensorConfig& config, AntennaLibrary* antLibrary);
	~Sensor();
	Sensor& operator = (const Sensor& sensor);
	RtLbsType GetGain() const;
	Point2D GetPosition2D() const;
	void AddSimulationError();			//增加仿真误差

};

inline void InitSensors(const SensorCollectionConfig& config, AntennaLibrary* antLibrary, std::vector<Sensor*>& sensors) {
	const std::vector<SensorConfig>& configs = config.m_sensorConfigs;
	sensors.resize(configs.size());
	for (int i = 0; i < configs.size(); ++i) {
		sensors[i] = new Sensor(configs[i], antLibrary);
		sensors[i]->m_id = i;
	}
}

#endif
