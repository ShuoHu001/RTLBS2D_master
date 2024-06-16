#ifndef RTLBS_PATHINFO
#define RTLBS_PATHINFO

#include "rtlbs.h"
#include "utility/define.h"
#include "math/complex.h"
#include "utility/enum.h"
#include "tree/raypath3d.h"
#include "tree/terraindiffractionpath.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "equipment/sensordata.h"
#include "equipment/sensor.h"


class PathInfo {
public:
	RAYPATHTYPE m_rayPathType;								/** @brief	·������	*/
	RtLbsType m_freq;										/** @brief	Ƶ��	  ��λ��Hz*/
	RtLbsType m_power;										/** @brief	����	 ��λ��dBm*/
	RtLbsType m_powerRatio;									/** @brief	����ռ��	*/
	RtLbsType m_scalarEField;								/** @brief	������ǿ	 ��λ��V/m*/
	Complex m_vectorEField;									/** @brief	ʸ����ǿ	 ��λ��V/m*/
	Complex m_magnitude;									/** @brief	������ǿ����������CFR��CIR��	*/
	RtLbsType m_timeDelay;									/** @brief	����ʱ��	��λ��ns*/
	RtLbsType m_phaseOffset;								/** @brief	��λ	ƫ�� ��λ������*/
	RtLbsType m_aoDPhi;										/** @brief	�뿪��(ˮƽ)	 ��λ������*/
	RtLbsType m_aoDTheta;									/** @brief	�뿪��(��ֱ)	 ��λ������*/
	RtLbsType m_aoAPhi;										/** @brief	�����(ˮƽ)	 ��λ������*/
	RtLbsType m_aoATheta;									/** @brief	�����(��ֱ)	 ��λ������*/

private:
	RayPath3D* m_rayPath;									/** @brief	����·��	*/
	TerrainDiffractionPath* m_terrainDiffractionPath;		/** @brief	��������·��	*/

public:
	PathInfo();
	PathInfo(const PathInfo& pathInfo);
	~PathInfo();
	PathInfo& operator = (const PathInfo& pathInfo);
	void SetRayPath(RayPath3D* path);
	void SetRayPath(TerrainDiffractionPath* path);
	void CalculateBaseInfo(RtLbsType freq, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, Transmitter* transmitter, Receiver* receiver);					//���������Ϣ-����׷��ģʽ
	void CalculateBaseInfo(RtLbsType power, RtLbsType freq, const AntennaLibrary* antLibrary, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, const Sensor* sensor);		//���������Ϣ-��λģʽ�е���������׷��
	void Convert2SensorData(SensorData& data) const;				//ת��Ϊ����������
};

//����PathInfo�е������Ĵ�С��������-�淽������
inline bool ComparedByPower_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_power > info2.m_power;
}

//����PathInfo�е�ʱ���С��������-�淽������
inline bool ComparedByDelay_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_timeDelay > info2.m_timeDelay;
}

//����PathInfo�еĵ���ǵ�phiֵ��������-�淽������
inline bool ComparedByAOAPhi_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoAPhi > info2.m_aoAPhi;
}

//����PathInfo�е��뿪�ǵ�phiֵ��������-�淽������
inline bool ComparedByAODPhi_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoDPhi > info2.m_aoDPhi;
}

//����PathInfo�еĵ���ǵ�thetaֵ��������-�淽������
inline bool ComparedByAOATheta_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoATheta > info2.m_aoATheta;
}

//����PathInfo�е��뿪�ǵ�thetaֵ��������-�淽������
inline bool ComparedByAODTheta_PathInfo(const PathInfo& info1, const PathInfo& info2) {
	return info1.m_aoDTheta > info2.m_aoDTheta;
}


#endif
