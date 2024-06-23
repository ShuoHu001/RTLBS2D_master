#include "sensordatalibrary.h"

SensorDataLibrary::SensorDataLibrary()
{
}

SensorDataLibrary::SensorDataLibrary(const SensorDataLibrary& library)
	: m_sensorDatas(library.m_sensorDatas)
	, m_sensorDataCollection(library.m_sensorDataCollection)
{
}

SensorDataLibrary::~SensorDataLibrary()
{
	for (auto it = m_sensorDatas.begin(); it != m_sensorDatas.end(); ++it) {
		delete* it;
	}

	for (auto it = m_sensorDataCollection.begin(); it != m_sensorDataCollection.end(); ++it) {
		delete* it;
	}
}

SensorDataLibrary& SensorDataLibrary::operator=(const SensorDataLibrary& library)
{
	m_sensorDatas = library.m_sensorDatas;
	m_sensorDataCollection = library.m_sensorDataCollection;
	return *this;
}

void SensorDataLibrary::AddNew(SensorDataCollection& collection)
{
	int sensorNum = static_cast<int>(m_sensorDataCollection.size());
	int dataNum = static_cast<int>(m_sensorDatas.size());
	collection.m_sensorId = sensorNum;														//在入库的同时修改原始数据的ID
	SensorDataCollection* newDataCollection = new SensorDataCollection(collection);
	m_sensorDataCollection.push_back(newDataCollection);
	for (int i = 0; i < collection.m_data.size(); ++i) {
		collection.m_data[i].m_id = dataNum++;												//修正传感器数据ID
		collection.m_data[i].m_sensorId = sensorNum;										//修正传感器数据所属传感器ID
		SensorData* newData = new SensorData(collection.m_data[i]);
		m_sensorDatas.push_back(newData);
	}
}

SensorDataCollection* SensorDataLibrary::GetDataCollection(int sensorId) const
{
	SensorDataCollection* reVal = nullptr;
	if (sensorId < m_sensorDataCollection.size() && sensorId >= 0) {
		reVal = m_sensorDataCollection[sensorId];
	}
	else {
		LOG_ERROR << "SensorDataLibrary: wrong sensorId: " << sensorId << " , max Id: " << m_sensorDataCollection.size() << " ." << ENDL;
	}
	return reVal;
}

SensorData* SensorDataLibrary::GetData(int dataId) const
{
	SensorData* reVal = nullptr;
	if (dataId < m_sensorDatas.size() && dataId >= 0) {
		reVal = m_sensorDatas[dataId];
	}
	else if (dataId == -2) {									//-2为TDOA对应的广义源获取方法
		return nullptr;
	}
	else {
		LOG_ERROR << "SensorDataLibrary: wrong dataId: " << dataId << " , max Id: " << m_sensorDatas.size() << " ." << ENDL;
	}
	return reVal;
}

void SensorDataLibrary::GetAllSensorDataCollection(std::vector<SensorDataCollection>& collections) const
{
	if (collections.size() != m_sensorDataCollection.size()) {
		collections.resize(m_sensorDataCollection.size());
	}
	for (size_t i = 0; i < m_sensorDataCollection.size(); ++i) {
		collections[i] = *m_sensorDataCollection[i];
	}
}

void SensorDataLibrary::GetAllSensorData(std::vector<SensorData>& datas) const
{
	if (datas.size() != m_sensorDatas.size()) {
		datas.resize(m_sensorDatas.size());
	}
	for (size_t i = 0; i < m_sensorDatas.size(); ++i) {
		datas[i] = *m_sensorDatas[i];
	}
}

