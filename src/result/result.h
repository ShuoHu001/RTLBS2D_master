#ifndef RTLBS_RESULT
#define RTLBS_RESULT
#include "math/point2d.h"
#include "radiowave/raypath/raypath.h"
#include "utility/define.h"
#include "raytracingresult.h"
#include "equipment/transceiver/transmitter.h"
#include "equipment/transceiver/receiver.h"
#include "equipment/sensor/sensor.h"
#include "configuration/outputconfig.h"
#include "configuration/frequencyconfig.h"
#include "material/materiallibrary.h"
#include "math/complex.h"
#include "equipment/sensor/sensordata.h"
#include "equipment/sensor/sensordatacollection.h"
#include "equipment/sensor/sensordataprocessing.h"
#include "configuration/sensor/sensorcollectionconfig.h"
#include "configuration/localization/lbserrorconfig.h"
#include "localization/localizationfunction/crlb.h"
#include "localization/localizationfunction/gdop.h"



class Result {

public:
	unsigned m_txNum;														/** @brief	���������	*/
	unsigned m_rxNum;														/** @brief	���ջ�����	*/
	unsigned m_sensorNum;													/** @brief	����������	*/
	std::vector<Transmitter*> m_transmitters;								/** @brief	���������	*/
	std::vector<Receiver*> m_receivers;										/** @brief	���ջ�����	*/
	std::vector<RaytracingResult> m_raytracingResult;						/** @brief	����׷�ٷ�����	*/

	std::vector<SensorDataCollection> m_sensorDataSPSTMD;					/** @brief	��վ��Դ�����ݶ�λ, ��������Ϊ1	  ֧�ֶ�λ���ͣ�TOA��TDOA��AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPSTSD;					/** @brief	��վ��Դ�����ݶ�λ, ����Ϊ���������*���ջ������� ֧�ֶ�λ���ͣ�TOA��TDOA��AOA*/
	std::vector<SensorDataCollection> m_sensorDataSPMTMD;					/** @brief	��վ��Դ�����ݶ�λ, ����Ϊ���ջ�������	֧�ֶ�λ���� AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPMTMD;					/** @brief	��վ��Դ�����ݶ�λ, ����Ϊ���ջ�������	֧�ֶ�λ���� AOA*/
private:
	mutable std::string m_directory;										/** @brief	�ļ��洢·��	*/
	SensorCollectionConfig m_sensorCollectionConfig;						/** @brief	RTģʽ��������������ݵ�ͬʱ����Ĵ����������ļ�	*/


public:
	Result();
	~Result();
	void Init(const std::vector<Transmitter*>& transmitters, const std::vector<Receiver*> receivers);										//��ʼ�����-����׷��
	void OutputResult(const OutputConfig& config);																					//���������
	void CalculateResult_RT_SensorData(const OutputConfig& outputConfig, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunctionData);																	//��������׷�ٽ��������ģ�⴫��������

private:
	void OutputVectorEField() const;																										//���ʸ����ų�
	void OutputScalarEField() const;																										//���������ų�
	void OutputVectorPower() const;																											//���ʸ������
	void OutputScalarPower() const;																											//�����������
	void OutputLoss() const;																												//������
	void OutputRayPath();																												//�������·��
	void OutputPDP() const;																													//�������ʱ����Ϣ
	void OutputCFR() const;																													//����ŵ�Ƶ����Ӧ��Ϣ
	void OutputCIR() const;																													//����ŵ��弤��Ӧ��Ϣ
	void OutputAOA() const;																													//����������Ϣ
	void OutputAOD() const;																													//����뿪����Ϣ
	void OutputSpreadProfile() const;																										//�����չ��Ϣ
	void OutputSensorDataSPSTMD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputSensorDataMPSTSD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputSensorDataSPMTMD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputSensorDataMPMTMD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputGeneralSourceForCRLB() const;																								//�������Դ��Ϣ�����ڼ���CRLB
	void OutputMultiStationCRLB(const LBSErrorConfig& config) const;																		//���㲢�����վCRLB
	void OutputMultiStationGDOP(const LBSErrorConfig& config) const;																		//���㲢�����վGDOP
	void OutputSingleStationCRLB(const LBSErrorConfig& config) const;																		//���㲢�����վCRLB
	void OutputSingleStationGDOP(const LBSErrorConfig& config) const;																		//���㲢�����վGDOP

	//��������Ҫ���������
	void OutputLocationRange() const;																										//�����λ��Χ
	
};

#endif
