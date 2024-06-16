#ifndef RTLBS_SENSORDATALIBRARY
#define RTLBS_SENSORDATALIBRARY

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "managers/logmanager.h"
#include "sensordata.h"


class SensorDataLibrary {
private:
	std::vector<SensorData*> m_sensorDatas;							/** @brief	�����������ݼ���	*/
	std::vector<SensorDataCollection*> m_sensorDataCollection;		/** @brief	��������Ӧ��ԭʼ���ݼ���	*/

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
