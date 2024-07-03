#ifndef RTLBS_GLOBALVARIABLES
#define RTLBS_GLOBALVARIABLES

#include "rtlbs.h"
#include "utility/define.h"
#include "math/complex.h"
#include "antenna/antenna.h"

extern std::vector<Complex> _global_tranFunctionData;									/** @brief	全局变量_计算绕射系数所用的传输函数（菲涅尔积分）	*/
extern std::vector<Antenna> _global_innerAntennas;										/** @brief	全局变量_计算时需要的内置天线方向图	*/
#endif
