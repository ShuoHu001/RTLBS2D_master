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
	std::vector<Sensor*> m_sensors;									/** @brief	ԭʼ�Ĵ�����ָ��	*/
	std::vector<SensorData*> m_sensorDatas;							/** @brief	�����������ݼ���	*/
	std::vector<SensorDataCollection*> m_sensorDataCollection;		/** @brief	��������Ӧ��ԭʼ���ݼ���	*/

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
	void GetAllSensorDataCollectionWithAOAError(std::vector<SensorDataCollection>& collections) const;										//��ȡ���д����������ݣ����մ������Ƕ����ϲ�
	void GetAllSensorDataCollectionWithTOAError(std::vector<SensorDataCollection>& collections) const;										//��ȡ���д����������ݣ����մ�����ʱ�����ϲ�
	void GetAllSensorData(std::vector<SensorData>& datas) const;
	void SetRandomPhiValue(RtLbsType deltaPhi);						//��������������仯��
};

#endif
