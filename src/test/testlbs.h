#ifndef RTLBS_TESTLBS
#define RTLBS_TESTLBS

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "system.h"
#include "rtresultsimilarity.h"


//测试单个站点的AOA定位误差
void TestAOALocalizationSingleStationInDifferentError();

//测试多站AOA定位误差
void TestAOAlocalizationMultiStationInDifferentError();

//测试单个站点的TOA定位误差
void TestTOALocalizationSingleStationInDifferentError(int positionId);

//测试多站TOA定位误差
void TestTOALocalizationMultiStationInDifferentError();

//测试单个站点的AOATDOA定位误差
void TestAOATDOALocalizationSingleStationInDifferentError();

//测试多站AOATDOA定位误差
void TestAOATDOALocalizationMultiStationInDifferentError();

//测试单个站点的AOATOA定位误差
void TestAOATOALocalizationSingleStationInDifferentError();

//测试多站AOATOA定位误差
void TestAOATOALocalizationMultiStationInDifferentError();

void TestAOALocalizaitonSingleStationErrorInDifferentPlace();

//研究平面上的定位误差分布情况
void ResearchMultipathSimilarityInLocalizationInDifferentPlaces();


//生成多站UWB定位功率数据
void GeneratePowerDataofUWBSystem();

//产生UWBAOA定位Sensor数据
void GenerateUWBAOALocalizationMultiStationSensorData();

//测试UWB-AOA定位
void TestUWBAOALocalizationMultiStation();

//测试UWB-TOA定位
void TestUWBTOALocalizationMultiStation();

//测试UWB-AOA/TOA定位
void TestUWBAOATOALocalizationMultiStation();

//测试UWB-AOA/TDOA定位
void TestUWBAOATDOALocalizationMultiStation();

//测试多个站点AOA定位，物体的位置发生改变的情况
void TestAOALocalizationMultiStationInGeometryError(int positionId);


//测试多个站点AOA定位，物体的位置发生改变的情况
void TestTOALocalizationMultiStationInGeometryError(int positionId);


//测试多个站点AOA定位，物体的位置发生改变的情况
void TestAOATOALocalizationMultiStationInGeometryError(int positionId);

//测试多个站点AOA定位，物体的位置发生改变的情况
void TestAOATDOALocalizationMultiStationInGeometryError(int positionId);

#endif
