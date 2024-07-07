#include "globalvariables.h"
#include "antenna/antenna.h"

std::vector<Complex> _global_tranFunctionData;							/** @brief	全局变量_计算绕射系数所用的传输函数（菲涅尔积分）	*/
std::vector<Antenna> _global_innerAntennas;								/** @brief	全局变量_计算时需要的内置天线方向图	*/
FrequencyConfig _global_freqConfig;