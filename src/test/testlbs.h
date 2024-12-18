#ifndef RTLBS_TESTLBS
#define RTLBS_TESTLBS

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "system.h"
#include "rtresultsimilarity.h"


//���Ե���վ���AOA��λ���
void TestAOALocalizationSingleStationInDifferentError();

//���Ե���վ���TOA��λ���
void TestTOALocalizationSingleStationInDifferentError();

//���Ե���վ���AOATDOA��λ���
void TestAOATDOALocalizationSingleStationInDifferentError();

//���Ե���վ���AOATOA��λ���
void TestAOATOALocalizationSingleStationInDifferentError();


void TestAOALocalizaitonSingleStationErrorInDifferentPlace();

//�о�ƽ���ϵĶ�λ���ֲ����
void ResearchMultipathSimilarityInLocalizationInDifferentPlaces();


#endif
