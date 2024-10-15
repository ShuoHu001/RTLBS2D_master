#include "lbsinfocluster.h"

LBSInfoCluster::LBSInfoCluster()
{
}

LBSInfoCluster::LBSInfoCluster(const std::vector<Sensor*>& sensors)
{
	m_infos.resize(sensors.size());
	for (int i = 0; i < static_cast<int>(sensors.size()); ++i) {
		m_infos[i] = new LBSInfo();
		m_infos[i]->m_sensor = sensors[i];
	}
}

LBSInfoCluster::~LBSInfoCluster()
{
	for (auto& curInfo : m_infos) {
		delete curInfo;
		curInfo = nullptr;
	}
}

void LBSInfoCluster::Init(const std::vector<Sensor*>& sensors)
{
	m_infos.resize(sensors.size());
	for (int i = 0; i < static_cast<int>(sensors.size()); ++i) {
		m_infos[i] = new LBSInfo();
		m_infos[i]->m_sensor = sensors[i];
	}
}

