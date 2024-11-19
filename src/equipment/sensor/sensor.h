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
	bool m_isValid;									/** @brief	�������Ƿ���Ч	*/
	int m_id;										/** @brief	������ID	*/
	Point3D m_position;								/** @brief	������λ��	*/
	Antenna* m_antenna;								/** @brief	����������	*/
	RtLbsType m_interLoss;							/** @brief	�������������	*/
	RtLbsType m_attachGain;							/** @brief	��������������	*/
	RtLbsType m_phiErrorSTD;						/** @brief	�������ǶȲ����ı�׼�� ��λ ����	*/
	RtLbsType m_timeErrorSTD;						/** @brief	������ʱ�Ӳ����ı�׼�� ��λ ns	*/
	RtLbsType m_powerErrorSTD;						/** @brief	���ʲ����ı�׼�� ��λ dB	*/
	SensorDataCollection m_sensorDataCollection;	/** @brief	�����������������ݼ���	*/

public:
	Sensor();
	Sensor(const Sensor& sensor);
	Sensor(const SensorConfig& config, AntennaLibrary* antLibrary);
	~Sensor();
	Sensor& operator = (const Sensor& sensor);
	RtLbsType GetGain() const;
	Point2D GetPosition2D() const;
	void AddSimulationError();			//���ӷ������

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
