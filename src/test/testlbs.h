#ifndef RTLBS_TESTLBS
#define RTLBS_TESTLBS

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "system.h"
#include "rtresultsimilarity.h"


//测试单个站点的AOA定位误差
void TestAOALocalizationSingleStationInDifferentError();

//测试单个站点的TOA定位误差
void TestTOALocalizationSingleStationInDifferentError();

//测试单个站点的AOATDOA定位误差
void TestAOATDOALocalizationSingleStationInDifferentError();

//测试单个站点的AOATOA定位误差
void TestAOATOALocalizationSingleStationInDifferentError();


void TestAOALocalizaitonSingleStationErrorInDifferentPlace();

//研究平面上的定位误差分布情况
void ResearchMultipathSimilarityInLocalizationInDifferentPlaces();


#endif
