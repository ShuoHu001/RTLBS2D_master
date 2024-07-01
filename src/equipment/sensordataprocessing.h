#ifndef RTLBS_SENSORDATAPROCESSING
#define RTLBS_SENSORDATAPROCESSING

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "sensordata.h"
#include "sensordatacollection.h"
#include "algorithm/decisiontoolkit/assignalgorithm.h"

//计算角度残差二范数
void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi);

//计算时间差残差二范数
void CalculateResidual_TDOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_timediff);

//计算时间差残差二范数和角度残差二范数
void CalculateResidual_TDOA_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_timediff);

//按照功率比较sensordatacollection的大小
bool ComparedByPower_SensorDataCollection(const SensorDataCollection& c1, const SensorDataCollection& c2);

//计算两个传感器数据的残差二范数-AOA定位方法-单数据
void CalculateSensorResidual_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi);

//计算两组传感器数据的残差二范数-AOA定位方法-单数据
void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff, int& nullDataNum);

//计算两个传感器数据的残差二范数-AOA定位方法-多数据
void CalculateSensorResidual_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_powerdiff, int nullDataNum);

//计算两组传感器数据的残差二范数-AOA定位算法-多数据
void CalculateSensorCollectionResidual_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff, int& nullDataNum);

//计算两个传感器数据的残差二范数-TDOA定位方法-单数据
void CalculateSensorResidual_TDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff);

//计算两组传感器数据的残差二范数-TDOA定位方法-单数据
void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum);

//计算两个传感器数据的残差二范数-TDOA定位方法-多数据
void CalculateSensorResidual_TDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum);

//计算两组传感器数据的残差二范数-TDOA定位方法-多数据
void CalculateSensorCollectionResidual_TDOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum);

//计算两个传感器数据的残差二范数-TDOA/AOA定位方法-单数据
void CalculateSensorResidual_TDOA_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff);

//计算两组传感器数据的残差二范数-TDOA/AOA定位方法-单数据
void CalculateSensorCollectionResidual_TDOA_AOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff);

//计算两个传感器数据的残差二范数-TDOA/AOA定位方法-多数据
void CalculateSensorResidual_TDOA_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff, int& nullDataNum);

//计算两组传感器数据的残差二范数-TDOA/AOA定位方法-多数据
void CalculateSensorCollectionResidual_TDOA_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff, int& nullDataNum);

#endif
