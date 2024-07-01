#ifndef RTLBS_RESULT
#define RTLBS_RESULT
#include "geometry/point2d.h"
#include "tree/raypath.h"
#include "utility/define.h"
#include "raytracingresult.h"
#include "equipment/transmitter.h"
#include "equipment/receiver.h"
#include "equipment/sensor.h"
#include "configuration/outputconfig.h"
#include "configuration/frequencyconfig.h"
#include "material/materiallibrary.h"
#include "math/complex.h"
#include "result/lbsresult.h"
#include "equipment/sensordata.h"
#include "equipment/sensordatacollection.h"
#include "equipment/sensordataprocessing.h"
#include "configuration/sensor/sensorcollectionconfig.h"
#include "localization/generalsource.h"
#include "localization/gspair.h"
#include "physical/pathbuilder.h"
#include "localization/aoa/aoasolver.h"
#include "physical/gpu/pathbuildergpu.h"
#include "localization/gspaircluster.h"
#include "localization/weightfactor.h"



class Result {

public:
	unsigned m_txNum;														/** @brief	���������	*/
	unsigned m_rxNum;														/** @brief	���ջ�����	*/
	unsigned m_sensorNum;													/** @brief	����������	*/
	std::vector<RaytracingResult> m_raytracingResult;						/** @brief	����׷�ٷ�����	*/
	std::vector<LBSResultGS> m_lbsGSResult;									/** @brief	��λ��� ����Դ	*/

	std::vector<SensorDataCollection> m_sensorDataSPSTMD;					/** @brief	��վ��Դ�����ݶ�λ, ��������Ϊ1	  ֧�ֶ�λ���ͣ�TOA��TDOA��AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPSTSD;					/** @brief	��վ��Դ�����ݶ�λ, ����Ϊ���������*���ջ������� ֧�ֶ�λ���ͣ�TOA��TDOA��AOA*/
	std::vector<SensorDataCollection> m_sensorDataSPMTMD;					/** @brief	��վ��Դ�����ݶ�λ, ����Ϊ���ջ�������	֧�ֶ�λ���� AOA*/
	std::vector<SensorDataCollection> m_sensorDataMPMTMD;					/** @brief	��վ��Դ�����ݶ�λ, ����Ϊ���ջ�������	֧�ֶ�λ���� AOA*/
private:
	mutable std::string m_directory;										/** @brief	�ļ��洢·��	*/
	std::vector<GeneralSource*> m_allGeneralSource;							/** @brief	���еĹ���Դ	*/
	SensorCollectionConfig m_sensorCollectionConfig;						/** @brief	RTģʽ��������������ݵ�ͬʱ����Ĵ����������ļ�	*/
	AOASolver m_aoaSolver;													/** @brief	AOA��λ�����	*/


public:
	Result();
	~Result();
	void Init(const std::vector<Transmitter*>& transmitters, const std::vector<Receiver*> receivers);										//��ʼ�����-����׷��
	void Init(const std::vector<Sensor*>& sensors, const std::vector<Receiver*>& receivers);												//��ʼ�����-��Դ��λ
	void Init(const std::vector<Sensor*>& sensors);																							//��ʼ�����-��Դ��λGS
	void OutputResult(SYSTEM_MODE systemMode, const OutputConfig& config) const;															//���������
	void CalculateResult_RT_SensorData(const FrequencyConfig& freqConfig, MaterialLibrary* matLibrary, const std::vector<Complex>& tranFunction, const OutputConfig& outputConfig);														//��������׷�ٽ��
	Point2D CalculateResult_LBS_AOA_MPSTSD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);
	Point2D CalculateResult_LBS_AOA_SPSTMD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);		//��������GS��λ����AOASPSTMD��λģʽ
	void CalculateResult_LBS_TDOA_MPSTSD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);		//��������GS��λ����TDOAMPSTSD��λģʽ
	void CalculateResult_LBS_TDOA_SPSTMD(HARDWAREMODE hardwareMode, const std::vector<RayTreeNode*>& vroots, const Scene* scene, RtLbsType splitRadius, LOCALIZATION_METHOD method, uint16_t threadNum, RtLbsType gsPairClusterThreshold, const WeightFactor& weightFactor, const FrequencyConfig& freqConfig, const std::vector<Complex>& tranFunction);		//��������GS��λ����TDOASPSTMD��λģʽ
	std::vector<GeneralSource*> GetGeneralSource() const;																					//��ù���Դ
	Point2D LocalizationSolver(const Point2D& initPoint);																												//��λ�����

private:
	void OutputVectorEField() const;																										//���ʸ����ų�
	void OutputScalarEField() const;																										//���������ų�
	void OutputVectorPower() const;																											//���ʸ������
	void OutputScalarPower() const;																											//�����������
	void OutputLoss() const;																												//������
	void OutputRayPath() const;																												//�������·��
	void OutputPDP() const;																													//�������ʱ����Ϣ
	void OutputCFR() const;																													//����ŵ�Ƶ����Ӧ��Ϣ
	void OutputCIR() const;																													//����ŵ��弤��Ӧ��Ϣ
	void OutputAOA() const;																													//����������Ϣ
	void OutputAOD() const;																													//����뿪����Ϣ
	void OutputSensorDataSPSTMD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputSensorDataMPSTSD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputSensorDataSPMTMD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputSensorDataMPMTMD() const;																									//�����վ��Դ�����ݶ�λ��������������
	void OutputGeneralSource() const;																										//�������Դ��Ϣ
	
};

#endif
