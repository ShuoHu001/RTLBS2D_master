#ifndef RTLBS_SENSOR
#define RTLBS_SENSOR

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/point3d.h"
#include "angularspectrum/angularspectrum.h"
#include "antenna/antenna.h"
#include "configuration/sensor/sensorconfig.h"
#include "configuration/sensor/sensorcollectionconfig.h"
#include "antenna/antennalibrary.h"
#include "sensordata.h"
#include "sensordatacollection.h"

class Sensor {
public:
	bool m_isValid;									/** @brief	�������Ƿ���Ч	*/
	int m_id;										/** @brief	������ID	*/
	Point3D m_position;								/** @brief	������λ��	*/
	Antenna* m_antenna;								/** @brief	����������	*/
	RtLbsType m_interLoss;							/** @brief	�������������	*/
	RtLbsType m_attachGain;							/** @brief	��������������	*/
	RtLbsType m_phiErrorSTD;						/** @brief	�������ǶȲ����ı�׼�� ��λ ����	*/
	RtLbsType m_timeErrorSTD;						/** @brief	������ʱ�Ӳ����ı�׼�� ��λ ns	*/
	SensorDataCollection m_sensorDataCollection;	/** @brief	�����������������ݼ���	*/

public:
	Sensor();
	Sensor(const Sensor& sensor);
	Sensor(const SensorConfig& config, AntennaLibrary* antLibrary);
	~Sensor();
	Sensor& operator = (const Sensor& sensor);
	RtLbsType GetGain() const;
	Point2D GetPosition2D() const;

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
