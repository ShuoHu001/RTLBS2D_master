#ifndef RTLBS_TESTLBS
#define RTLBS_TESTLBS

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "system.h"
#include "rtresultsimilarity.h"


//���Ե���վ���AOA��λ���
void TestAOALocalizationSingleStationInDifferentError();

//���Զ�վAOA��λ���
void TestAOAlocalizationMultiStationInDifferentError();

//���Ե���վ���TOA��λ���
void TestTOALocalizationSingleStationInDifferentError(int positionId);

//���Զ�վTOA��λ���
void TestTOALocalizationMultiStationInDifferentError();

//���Ե���վ���AOATDOA��λ���
void TestAOATDOALocalizationSingleStationInDifferentError();

//���Զ�վAOATDOA��λ���
void TestAOATDOALocalizationMultiStationInDifferentError();

//���Ե���վ���AOATOA��λ���
void TestAOATOALocalizationSingleStationInDifferentError();

//���Զ�վAOATOA��λ���
void TestAOATOALocalizationMultiStationInDifferentError();


//���Ե�վƽ�涨λ����_AOA��λ�㷨
void TestAOALocalizaitonSingleStationErrorInDifferentPlace();

//���Ե�վƽ�涨λ����_TOA��λ�㷨
void TestTOALocalizaitonSingleStationErrorInDifferentPlace();

//���Ե�վƽ�涨λ����_AOATDOA��λ�㷨
void TestAOATDOALocalizaitonSingleStationErrorInDifferentPlace();


//���ɶ�վUWB��λ��������
void GeneratePowerDataofUWBSystem();

//����UWBAOA��λSensor����
void GenerateUWBAOALocalizationMultiStationSensorData();

//����UWB-AOA��λ
void TestUWBAOALocalizationMultiStation();

//����UWB-TOA��λ
void TestUWBTOALocalizationMultiStation();

//����UWB-AOA/TOA��λ
void TestUWBAOATOALocalizationMultiStation();

//����UWB-AOA/TDOA��λ
void TestUWBAOATDOALocalizationMultiStation();

//���Զ��վ��AOA��λ�������λ�÷����ı�����
void TestAOALocalizationMultiStationInGeometryError(int positionId);


//���Զ��վ��AOA��λ�������λ�÷����ı�����
void TestTOALocalizationMultiStationInGeometryError(int positionId);


//���Զ��վ��AOA��λ�������λ�÷����ı�����
void TestAOATOALocalizationMultiStationInGeometryError(int positionId);

//���Զ��վ��AOA��λ�������λ�÷����ı�����
void TestAOATDOALocalizationMultiStationInGeometryError(int positionId);

#endif
