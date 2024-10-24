#ifndef RTLBS_SENSORDATALIBRARY
#define RTLBS_SENSORDATALIBRARY

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "managers/logmanager.h"
#include "sensor.h"
#include "sensordata.h"
#include "sensordatacollection.h"
#include "managers/randomanager.h"


class SensorDataLibrary {
private:
	std::vector<Sensor*> m_sensors;									/** @brief	原始的传感器指针	*/
	std::vector<SensorData*> m_sensorDatas;							/** @brief	传感器的数据集合	*/
	std::vector<SensorDataCollection*> m_sensorDataCollection;		/** @brief	传感器对应的原始数据集合	*/

public:
	SensorDataLibrary();
	SensorDataLibrary(const SensorDataLibrary& library);
	~SensorDataLibrary();
	SensorDataLibrary& operator = (const SensorDataLibrary& library);
	void AddSensor(Sensor* sensor);
	void AddNew(SensorDataCollection& collection);
	SensorDataCollection* GetDataCollection(int sensorId) const;
	SensorData* GetData(int dataId) const;
	void GetAllSensorDataCollection(std::vector<SensorDataCollection>& collections) const;
	void GetAllSensorDataCollectionWithAOAError(std::vector<SensorDataCollection>& collections) const;										//获取所有传感器的数据，按照传感器角度误差合并
	void GetAllSensorDataCollectionWithTOAError(std::vector<SensorDataCollection>& collections) const;										//获取所有传感器的数据，按照传感器时间误差合并
	void GetAllSensorData(std::vector<SensorData>& datas) const;
	void SetRandomPhiValue(RtLbsType deltaPhi);						//对数据设置随机变化量
};

#endif
