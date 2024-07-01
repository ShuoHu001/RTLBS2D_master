#ifndef RTLBS_SENSORDATALIBRARY
#define RTLBS_SENSORDATALIBRARY

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "managers/logmanager.h"
#include "sensordata.h"
#include "sensordatacollection.h"
#include "managers/randomanager.h"


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
	void GetAllSeneorDataCollectionWithAOAPhiError(std::vector<SensorDataCollection>& collections, RtLbsType phiErrorSTD, RtLbsType powerErrorSTD) const;		//获取所有传感器的数据，给定噪声
	void GetAllSensorData(std::vector<SensorData>& datas) const;
	void SetRandomPhiValue(RtLbsType deltaPhi);						//对数据设置随机变化量
};

#endif
