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
	int m_pathNum;										/** @brief	·������	*/
	int m_freqNum;										/** @brief	Ƶ������	*/
	Sensor* m_sensor;									/** @brief	������	*/
	Receiver* m_receiver;								/** @brief	���ջ�	*/
	std::vector<RtLbsType> m_freqs;						/** @brief	�����Ƶ��	*/
	std::vector<RayPath3D*> m_paths;					/** @brief	����·��	*/
	std::vector<RtLbsType> m_emittedPower;				/** @brief	���Ƶõ��ķ��书��dBm	*/
	int m_featureSize;									/** @brief	�õ�����������	*/
	RtLbsType m_powerSTDValue;							/** @brief	�õ��ķ��书�ʾ�ֵƽ����(��׼��)	*/
public:
	LBSResult();
	~LBSResult();
	void SetRayPath(std::vector<RayPath3D*>& paths);				//����·����Ϣ
	void CalculateBaseInfo(std::vector<RtLbsType>& freqs, MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction);		//���������Ϣ
};


//LBS�еĹ���Դ�汾����
class LBSResultGS {
public:
	Sensor* m_sensor;
	std::vector<LBSTreeNode*> m_nodes;
	std::vector<GeneralSource*> m_sources;

public:
	LBSResultGS();
	~LBSResultGS();
	void SetNodes(std::vector<LBSTreeNode*>& nodes);
	void CalculateBaseInfo(LOCALIZATION_METHOD lbsMethod);								//���������Ϣ
};

inline void EraseRepeatGeneralSources(std::vector<GeneralSource*>& sources);

#endif
