#ifndef RTLBS_RAYTRACINGRESULT
#define RTLBS_RAYTRACINGRESULT


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "equipment/sensor.h"
#include "tree/raypath3d.h"
#include "tree/terraindiffractionpath.h"
#include "pathinfo.h"
#include "math/complex.h"
#include "material/material.h"
#include "material/materiallibrary.h"
#include "equipment/sensordata.h"

class RaytracingResult {
public:
	int m_pathNum;											/** @brief	·������	*/
	int m_freqNum;											/** @brief	Ƶ������	*/
	Transmitter* m_transmitter;								/** @brief	�����	*/
	Receiver* m_receiver;									/** @brief	���ջ�	*/
	std::vector<RtLbsType> m_freqs;							/** @brief	�����Ƶ��	*/
	std::vector<RayPath3D*> m_commonPaths;					/** @brief	����·��	*/
	TerrainDiffractionPath* m_terrainDiffPath;				/** @brief	��������·��	*/

	std::vector<PathInfo> m_multipathInfo;					/** @brief	�ྶ��Ϣ	*/
	std::vector<Complex> m_totalVectorEField;				/** @brief	�ϳ�ʸ���糡	*/
	std::vector<RtLbsType> m_totalScalarEField;				/** @brief	�ϳɱ����糡	*/
	std::vector<RtLbsType> m_vectorPower;					/** @brief	ʸ�����չ���	*/
	std::vector<RtLbsType> m_scalarPower;					/** @brief	�������չ���	*/
	std::vector<RtLbsType> m_loss;							/** @brief	���	*/
	std::vector<Complex> m_magnitudesCFR;					/** @brief	�ŵ�Ƶ����ӦCFR	*/
	std::vector<Complex> m_magnitudesCIR;					/** @brief	�ŵ��弤��ӦCIR	*/


public:
	RaytracingResult();
	RaytracingResult(const RaytracingResult& result);
	~RaytracingResult();
	RaytracingResult& operator = (const RaytracingResult& result);


	void SetRayPath(std::vector<RayPath3D*>& paths);		//����·����Ϣ ����·��
	void SetRayPath(TerrainDiffractionPath* path);			//����·����Ϣ ��������
	void CalculateBaseInfo(std::vector<RtLbsType>& freqs, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction);								//���������Ϣ-����׷��ģʽ
	void CalculateBaseInfo(const Sensor* sensor, std::vector<RtLbsType>& freqs, const AntennaLibrary* antLibrary, const MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction);		//���������Ϣ-��λģʽ�еİ�������׷��
	void GetAllSensorData(SensorDataCollection& collection) const;			//��ȡ���еĴ��������ݣ�ת����
	void GetAllSensorData(SensorDataCollection* collection) const;			//��ȡ���еĴ���������		ָ����
	void GetMaxPowerSensorData(SensorDataCollection& collection) const;		//��ȡ����ʵĴ��������� ��ת����
	void GetMaxPowerSensorData(SensorDataCollection* collection) const;		//��ȡ����ʵĴ��������� ָ����
	void OutputVectorEField(std::ofstream& stream) const;																					//���ʸ����
	void OutputScalarEField(std::ofstream& stream) const;																					//���������
	void OutputVectorPower(std::ofstream& stream) const;																					//���ʸ��������Ϣ
	void OutputScalarPower(std::ofstream& stream) const;																					//�������������Ϣ
	void OutputLoss(std::ofstream& stream) const;																							//��������Ϣ
	void OutputRayPath(std::ofstream& stream) const;																						//����ྶ��Ϣ
	void OutputPDP(std::ofstream& stream) const;																							//���PDP��Ϣ
	void OutputCFR(std::ofstream& stream) const;																							//���CFR��Ϣ
	void OutputCIR(std::ofstream& stream) const;																							//���CIR��Ϣ
	void OutputAOA(std::ofstream& stream) const;																							//���AOA��Ϣ
	void OutputAOD(std::ofstream& stream) const;																							//���AOD��Ϣ
};

#endif
