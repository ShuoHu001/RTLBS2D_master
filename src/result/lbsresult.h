#ifndef RTLBS_LOCATIONBASEDSERVICE
#define RTLBS_LOCATIONBASEDSERVICE

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "equipment/sensor.h"
#include "equipment/receiver.h"
#include "tree/raypath3d.h"
#include "tree/lbstreenode.h"
#include "localization/generalsource.h"

class LBSResult {
public:
	int m_pathNum;										/** @brief	路径数量	*/
	int m_freqNum;										/** @brief	频率数量	*/
	Sensor* m_sensor;									/** @brief	传感器	*/
	Receiver* m_receiver;								/** @brief	接收机	*/
	std::vector<RtLbsType> m_freqs;						/** @brief	计算的频点	*/
	std::vector<RayPath3D*> m_paths;					/** @brief	射线路径	*/
	std::vector<RtLbsType> m_emittedPower;				/** @brief	逆推得到的辐射功率dBm	*/
	int m_featureSize;									/** @brief	得到的特征数量	*/
	RtLbsType m_powerSTDValue;							/** @brief	得到的辐射功率均值平滑度(标准差)	*/
public:
	LBSResult();
	~LBSResult();
	void SetRayPath(std::vector<RayPath3D*>& paths);				//设置路径信息
	void CalculateBaseInfo(std::vector<RtLbsType>& freqs, MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction);		//计算基本信息
};


//LBS中的广义源版本数据
class LBSResultGS {
public:
	Sensor* m_sensor;
	std::vector<LBSTreeNode*> m_nodes;
	std::vector<GeneralSource*> m_sources;

public:
	LBSResultGS();
	~LBSResultGS();
	void SetNodes(std::vector<LBSTreeNode*>& nodes);
	void CalculateBaseInfo(LOCALIZATION_METHOD lbsMethod);								//计算基本信息
};

inline void EraseRepeatGeneralSources(std::vector<GeneralSource*>& sources);

#endif
