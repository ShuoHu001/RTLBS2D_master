#include "globalvariables.h"
#include "antenna/antenna.h"

std::vector<Complex> _global_tranFunctionData;							/** @brief	全局变量_计算绕射系数所用的传输函数（菲涅尔积分）	*/
std::vector<Antenna> _global_innerAntennas;								/** @brief	全局变量_计算时需要的内置天线方向图	*/
FrequencyConfig _global_freqConfig;										/** @brief	全局变量_计算时需要的频率参数	*/
int _global_diffractRayNum;												/** @brief	全局变量_计算绕射时离散的数量	*/
ElevationMatrix _global_lbsShiftErrorMatrix;							/** @brief	全局变量_定位服务提高定位精度的位移变化误差矩阵	*/