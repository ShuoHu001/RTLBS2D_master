#ifndef RTLBS_RECEIVER
#define RTLBS_RECEIVER

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/point3d.h"
#include "antenna/antenna.h"
#include "configuration/receiver/receiverconfig.h"
#include "configuration/receiver/receiverunitconfig.h"
#include "antenna/antennalibrary.h"

class Receiver {
public:
	bool m_isValid;					/** @brief	接收机是否有效	*/
	int m_id;						/** @brief	接收天线ID	*/
	Antenna* m_antenna;				/** @brief	接收机所用天线	*/
	Point3D m_position;				/** @brief	天线位置信息	*/
	Euler m_posture;				/** @brief	天线姿态信息	*/
	RtLbsType m_velocity;			/** @brief	天线速度信息	*/
	RtLbsType m_interLoss;			/** @brief	天线插入损耗	*/
	RtLbsType m_attachGain;			/** @brief	接收机附加增益	*/
	RtLbsType m_powerThreshold;		/** @brief	接收机接收电平灵敏度	*/
	RtLbsType m_angularThreshold;	/** @brief	接收机接收角度的灵敏度, 单位:弧度	*/
	RtLbsType m_delayThreshold;		/** @brief	接收机接收时延的灵敏度, 单位:s	*/

public:
	Receiver();
	Receiver(const Receiver& re);
	Receiver(const ReceiverUnitConfig& config, AntennaLibrary* antLibrary);
	~Receiver();
	Receiver& operator = (const Receiver& re);
	RtLbsType GetGain() const;				//获取接收机的增益
	Point3D GetPosition3D() const;			//获取接收机的三维位置
	Point2D GetPosition2D() const;			//获取接收机的二维位置
};

inline void InitReceiver(const std::vector<ReceiverUnitConfig>& configs, AntennaLibrary* antLibrary, std::vector<Receiver*>& receivers) {
	int reNum = static_cast<int>(configs.size());
	receivers.resize(reNum);
	for (int i = 0; i < reNum; ++i) {
		receivers[i] = new Receiver(configs[i], antLibrary);
		receivers[i]->m_id = i;
	}
}

#endif
