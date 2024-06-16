#ifndef RTLBS_RECEIVERCONFIG
#define RTLBS_RECEIVERCONFIG

#include "rtlbs.h"
#include "utility/define.h"
#include "singleconfig.h"
#include "lineconfig.h"
#include "scatterconfig.h"
#include "planeconfig.h"
#include "antenna/antenna.h"
#include "utility/enum.h"
#include "utility/define.h"
#include "geometry/point3d.h"
#include "solidconfig.h"
#include "receiverunitconfig.h"


const std::string KEY_RECEIVERCONFIG_PREDICTIONMODE = "PredictionMode";
const std::string KEY_RECEIVERCONFIG_SINGLECONFIG = "SingleConfig";
const std::string KEY_RECEIVERCONFIG_LINECONFIG = "LineConfig";
const std::string KEY_RECEIVERCONFIG_SCATTERCONFIG = "ScatterConfig";
const std::string KEY_RECEIVERCONFIG_PLANECONFIG = "PlaneConfig";
const std::string KEY_RECEIVERCONFIG_SOLIDCONFIG = "SolidConfig";
const std::string KEY_RECEIVERCONFIG_ATTACHFILENAME = "AttachFileName";
const std::string KEY_RECEIVERCONFIG_ANTID = "AntId";
const std::string KEY_RECEIVERCONFIG_ANTNAME = "AntName";
const std::string KEY_RECEIVERCONFIG_INSERTLOSS = "InsertLoss";
const std::string KEY_RECEIVERCONFIG_ATTACHGAIN = "AttachGain";
const std::string KEY_RECEIVERCONFIG_POWERTHRESHOLD = "PowerThreshold";
const std::string KEY_RECEIVERCONFIG_POSTURE = "Posture";
const std::string KEY_RECEIVERCONFIG_RECEIVERS = "Receivers";
const std::string KEY_RECEIVERCONFIG = "ReceiverConfig";

class ReceiverConfig : public Serializable {

public:
	PREDICTION_MODE m_predictionMode;	/** @brief	Ԥ��ģʽ	*/

	SingleConfig m_singleConfig;		/** @brief	����Ԥ��ģʽ	*/
	LineConfig m_lineConfig;			/** @brief	�߶�Ԥ��ģʽ	*/
	ScatterConfig m_scatterConfig;		/** @brief	��ɢ��Ԥ��ģʽ	*/
	PlaneConfig m_planeConfig;			/** @brief	ƽ��Ԥ��ģʽ	*/
	SolidConfig m_solidConfig;			/** @brief	����Ԥ��ģʽ	*/
	std::string m_attachFileName;		/** @brief	�ļ�ģʽ��ȡʱ���ļ�����	*/
	unsigned m_antId;					/** @brief	����Id	*/
	std::string m_antName;				/** @brief	��������	*/
	RtLbsType m_insertLoss;				/** @brief	���ջ��������	*/
	RtLbsType m_attachGain;				/** @brief	���ջ���������	*/
	RtLbsType m_powerShreshold;			/** @brief	���ջ����ʶ���ʵ�ƽ, ��λ:dBm	*/
	Euler m_posture;					/** @brief	���ջ���̬	*/


	//�����л�����
	std::vector<ReceiverUnitConfig> m_receiverconfigs;			/** @brief	���ջ�����	*/

public:
	ReceiverConfig();
	ReceiverConfig(const ReceiverConfig& config);
	~ReceiverConfig();
	ReceiverConfig& operator = (const ReceiverConfig& config);
	void CalculateRxPositions();  //���յ�λ��ʼ��
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	bool InitReceiversFromFile(std::string filename);									//���ļ��н��г�ʼ�����ջ�
	bool Init(const std::string& filename);												//��ʼ������
	void Write2Json(const std::string& filename);										//���л����ļ���
};


#endif
