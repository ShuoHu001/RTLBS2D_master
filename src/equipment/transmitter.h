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
	bool m_isValid;						/** @brief	������Ƿ���Ч	*/
	int m_id;							/** @brief	�����ID��ȫ��Ψһ��	*/
	Antenna* m_antenna;					/** @brief	�����������������	*/
	Point3D m_position;					/** @brief	���������λ��(��ά)	*/
	Euler m_posture;					/** @brief	�����������̬��Ϣ	*/
	RtLbsType m_velocity;				/** @brief	��������ٶ�	*/
	RtLbsType m_power;					/** @brief	����	, ��λ��W*/
	RtLbsType m_interLoss;				/** @brief	�������	*/
	RtLbsType m_attachGain;				/** @brief	�������������	*/
public:
	Transmitter();
	Transmitter(const TransmitterConfig& config, AntennaLibrary* antLibrary);
	Transmitter(const Transmitter& tr);
	~Transmitter();
	Transmitter& operator = (const Transmitter& tr);
	RtLbsType GetGain() const;						//��ȡ����ϵͳ������
	Point3D GetPosition3D() const;					//��ȡ�������ߵ���άλ��
	Point2D GetPosition2D() const;					//��ȡ�������ߵĶ�άλ��
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
