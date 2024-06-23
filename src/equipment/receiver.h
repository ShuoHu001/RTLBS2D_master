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
	bool m_isValid;					/** @brief	���ջ��Ƿ���Ч	*/
	int m_id;						/** @brief	��������ID	*/
	Antenna* m_antenna;				/** @brief	���ջ���������	*/
	Point3D m_position;				/** @brief	����λ����Ϣ	*/
	Euler m_posture;				/** @brief	������̬��Ϣ	*/
	RtLbsType m_velocity;			/** @brief	�����ٶ���Ϣ	*/
	RtLbsType m_interLoss;			/** @brief	���߲������	*/
	RtLbsType m_attachGain;			/** @brief	���ջ���������	*/
	RtLbsType m_powerThreshold;		/** @brief	���ջ����յ�ƽ������	*/
	RtLbsType m_angularThreshold;	/** @brief	���ջ����սǶȵ�������, ��λ:����	*/
	RtLbsType m_delayThreshold;		/** @brief	���ջ�����ʱ�ӵ�������, ��λ:s	*/

public:
	Receiver();
	Receiver(const Receiver& re);
	Receiver(const ReceiverUnitConfig& config, AntennaLibrary* antLibrary);
	~Receiver();
	Receiver& operator = (const Receiver& re);
	RtLbsType GetGain() const;				//��ȡ���ջ�������
	Point3D GetPosition3D() const;			//��ȡ���ջ�����άλ��
	Point2D GetPosition2D() const;			//��ȡ���ջ��Ķ�άλ��
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
