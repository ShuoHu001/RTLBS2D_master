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
	PREDICTION_MODE m_predictionMode;	/** @brief	预测模式	*/

	SingleConfig m_singleConfig;		/** @brief	单点预测模式	*/
	LineConfig m_lineConfig;			/** @brief	线段预测模式	*/
	ScatterConfig m_scatterConfig;		/** @brief	离散点预测模式	*/
	PlaneConfig m_planeConfig;			/** @brief	平面预测模式	*/
	SolidConfig m_solidConfig;			/** @brief	立体预测模式	*/
	std::string m_attachFileName;		/** @brief	文件模式读取时的文件名称	*/
	unsigned m_antId;					/** @brief	天线Id	*/
	std::string m_antName;				/** @brief	天线名称	*/
	RtLbsType m_insertLoss;				/** @brief	接收机插入损耗	*/
	RtLbsType m_attachGain;				/** @brief	接收机附加增益	*/
	RtLbsType m_powerShreshold;			/** @brief	接收机最低识别功率电平, 单位:dBm	*/
	Euler m_posture;					/** @brief	接收机姿态	*/


	//非序列化对象
	std::vector<ReceiverUnitConfig> m_receiverconfigs;			/** @brief	接收机配置	*/

public:
	ReceiverConfig();
	ReceiverConfig(const ReceiverConfig& config);
	~ReceiverConfig();
	ReceiverConfig& operator = (const ReceiverConfig& config);
	void CalculateRxPositions();  //接收点位初始化
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
	bool InitReceiversFromFile(std::string filename);									//从文件中进行初始化接收机
	bool Init(const std::string& filename);												//初始化配置
	void Write2Json(const std::string& filename);										//序列化至文件中
};


#endif
