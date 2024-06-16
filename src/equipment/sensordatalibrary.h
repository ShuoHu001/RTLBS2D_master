#ifndef RTLBS_SENSORDATALIBRARY
#define RTLBS_SENSORDATALIBRARY

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "managers/logmanager.h"
#include "sensordata.h"


class SensorDataLibrary {
private:
	std::vector<SensorData*> m_sensorDatas;							/** @brief	传感器的数据集合	*/
	std::vector<SensorDataCollection*> m_sensorDataCollection;		/** @brief	传感器对应的原始数据集合	*/

public:
	SensorDataLibrary();
	SensorDataLibrary(const SensorDataLibrary& library);
	~SensorDataLibrary();
	SensorDataLibrary& operator = (const SensorDataLibrary& library);
	void AddNew(SensorDataCollection& collection);
	SensorDataCollection* GetDataCollection(int sensorId) const;
	SensorData* GetData(int dataId) const;
	void GetAllSensorDataCollection(std::vector<SensorDataCollection>& collections) const;
	void GetAllSensorData(std::vector<SensorData>& datas) const;
};

#endif
