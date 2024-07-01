#ifndef RTLBS_SENSORDATAPROCESSING
#define RTLBS_SENSORDATAPROCESSING

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "sensordata.h"
#include "sensordatacollection.h"
#include "algorithm/decisiontoolkit/assignalgorithm.h"

//����ǶȲв������
void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi);

//����ʱ���в������
void CalculateResidual_TDOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_timediff);

//����ʱ���в�������ͽǶȲв������
void CalculateResidual_TDOA_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_timediff);

//���չ��ʱȽ�sensordatacollection�Ĵ�С
bool ComparedByPower_SensorDataCollection(const SensorDataCollection& c1, const SensorDataCollection& c2);

//�����������������ݵĲв������-AOA��λ����-������
void CalculateSensorResidual_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi);

//�������鴫�������ݵĲв������-AOA��λ����-������
void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff, int& nullDataNum);

//�����������������ݵĲв������-AOA��λ����-������
void CalculateSensorResidual_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_powerdiff, int nullDataNum);

//�������鴫�������ݵĲв������-AOA��λ�㷨-������
void CalculateSensorCollectionResidual_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff, int& nullDataNum);

//�����������������ݵĲв������-TDOA��λ����-������
void CalculateSensorResidual_TDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff);

//�������鴫�������ݵĲв������-TDOA��λ����-������
void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum);

//�����������������ݵĲв������-TDOA��λ����-������
void CalculateSensorResidual_TDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum);

//�������鴫�������ݵĲв������-TDOA��λ����-������
void CalculateSensorCollectionResidual_TDOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum);

//�����������������ݵĲв������-TDOA/AOA��λ����-������
void CalculateSensorResidual_TDOA_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff);

//�������鴫�������ݵĲв������-TDOA/AOA��λ����-������
void CalculateSensorCollectionResidual_TDOA_AOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff);

//�����������������ݵĲв������-TDOA/AOA��λ����-������
void CalculateSensorResidual_TDOA_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum);

//�������鴫�������ݵĲв������-TDOA/AOA��λ����-������
void CalculateSensorCollectionResidual_TDOA_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum);

#endif
