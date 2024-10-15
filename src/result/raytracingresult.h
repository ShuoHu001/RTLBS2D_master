#ifndef RTLBS_RAYTRACINGRESULT
#define RTLBS_RAYTRACINGRESULT


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "equipment/transceiver/transmitter.h"
#include "equipment/transceiver/receiver.h"
#include "equipment/sensor/sensor.h"
#include "radiowave/raypath/raypath3d.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "pathinfo.h"
#include "pathinfocluster.h"
#include "math/complex.h"
#include "material/material.h"
#include "material/materiallibrary.h"
#include "equipment/sensor/sensordata.h"



class RaytracingResult {
public:
	bool m_isValid;											/** @brief	����Ƿ���Ч	*/
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
	RtLbsType m_rmsDelaySpread;								/** @brief	������ʱ����չ	*/
	RtLbsType m_rmsAngularSpread;							/** @brief	�������Ƕ���չ	*/

public:
	RaytracingResult();
	RaytracingResult(const RaytracingResult& result);
	~RaytracingResult();
	RaytracingResult& operator = (const RaytracingResult& result);


	void SetRayPath(std::vector<RayPath3D*>& paths);		//����·����Ϣ ����·��
	void SetRayPath(TerrainDiffractionPath* path);			//����·����Ϣ ��������
	void ReleaseAllRayPath();								//�ͷ���������·����Ϣ
	void CalculateBaseInfo(std::vector<RtLbsType>& freqs, const std::vector<Complex>& tranFunctionData);								//���������Ϣ-����׷��ģʽ
	
	void CalculateBaseInfo(const Sensor* sensor, std::vector<RtLbsType>& freqs, const std::vector<Complex>& tranFunctionData, const AntennaLibrary* antLibrary);		//���������Ϣ-��λģʽ�еİ�������׷��
	void GetAllSensorData_AOA2D(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const;						//��ȡ���еĴ���������,������AOA2D��λ
	void GetMaxPowerSensorData_AOA2D(SensorDataCollection& collection, RtLbsType threshold) const;											//��ȡ����ʵĴ���������,������AOA2D��λ
	void GetAllSensorData_AOA3D(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const;						//��ȡ���еĴ���������,������AOA3D��λ
	void GetMaxPowerSensorData_AOA3D(SensorDataCollection& collection, RtLbsType threshold) const;											//��ȡ����ʵĴ���������,������AOA3D��λ
	void GetAllSensorData_Delay(SensorDataCollection& collection, RtLbsType threshold, RtLbsType sparseFactor) const;						//��ȡ���еĴ���������,������ʱ���Ͷ�λ
	void GetMaxPowerSensorData_Delay(SensorDataCollection& collection, RtLbsType threshold) const;											//��ȡ����ʵĴ���������,������ʱ���Ͷ�λ
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
	void OutputSpreadProfile(std::ofstream& stream) const;																					//�����չ�����Ϣ������ʱ����չ�ͽǶ���չ
	void OutputGeneralSourceForCRLB(std::ofstream& stream) const;																			//�������Դ��Ϣ��Ϊ�˼���CRLB

private:
	RtLbsType CalculateMeanArrivedDelay() const;																							//����ƽ������ʱ��
	RtLbsType CalculateMeanArrivedAngle() const;																							//����ƽ������Ƕ�
	RtLbsType CalculateRMSDelaySpread() const;																								//���������ʱ����չ
	RtLbsType CalculateRMSAngularSpread() const;																							//����������Ƕ���չ

};

#endif
