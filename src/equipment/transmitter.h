#ifndef RTLBS_TRANSMITTER
#define RTLBS_TRANSMITTER

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/point3d.h"
#include "antenna/antenna.h"
#include "configuration/transmitter/transmitterconfig.h"
#include "configuration/transmitter/transmittercollectionconfig.h"
#include "antenna/antennalibrary.h"


class Transmitter {
public:
	bool m_isValid;						/** @brief	发射机是否有效	*/
	int m_id;							/** @brief	发射机ID（全局唯一）	*/
	Antenna* m_antenna;					/** @brief	发射机所用天线天线	*/
	Point3D m_position;					/** @brief	发射机所在位置(三维)	*/
	Euler m_posture;					/** @brief	发射机天线姿态信息	*/
	RtLbsType m_velocity;				/** @brief	发射机的速度	*/
	RtLbsType m_power;					/** @brief	功率	, 单位：W*/
	RtLbsType m_interLoss;				/** @brief	插入损耗	*/
	RtLbsType m_attachGain;				/** @brief	发射机附加增益	*/
public:
	Transmitter();
	Transmitter(const TransmitterConfig& config, AntennaLibrary* antLibrary);
	Transmitter(const Transmitter& tr);
	~Transmitter();
	Transmitter& operator = (const Transmitter& tr);
	RtLbsType GetGain() const;						//获取发射系统的增益
	Point3D GetPosition3D() const;					//获取发射天线的三维位置
	Point2D GetPosition2D() const;					//获取发射天线的二维位置
};

inline void InitTransmitter(const TransmitterCollectionConfig& config, AntennaLibrary* antLibrary, std::vector<Transmitter*>& transmitters) {
	const std::vector<TransmitterConfig>& configs = config.m_transmitterConfigs;
	transmitters.resize(configs.size());
	for (int i = 0; i < configs.size(); ++i) {
		transmitters[i] = new Transmitter(configs[i], antLibrary);
		transmitters[i]->m_id = i;
	}
}

inline void InitTransmitter(const std::vector<TransmitterConfig>& configs, AntennaLibrary* antLibrary, std::vector<Transmitter*>& transmitters) {
	transmitters.resize(configs.size());
	for (int i = 0; i < configs.size(); ++i) {
		transmitters[i] = new Transmitter(configs[i], antLibrary);
		transmitters[i]->m_id = i;
	}
}

#endif
