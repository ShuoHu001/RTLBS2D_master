#ifndef RTLBS_GLOBALVARIABLES
#define RTLBS_GLOBALVARIABLES

#include "rtlbs.h"
#include "utility/define.h"
#include "math/complex.h"
#include "configuration/frequencyconfig.h"

class Antenna;


extern std::vector<Complex> _global_tranFunctionData;									/** @brief	ȫ�ֱ���_��������ϵ�����õĴ��亯�������������֣�	*/
extern std::vector<Antenna> _global_innerAntennas;										/** @brief	ȫ�ֱ���_����ʱ��Ҫ���������߷���ͼ	*/
extern FrequencyConfig _global_freqConfig;												/** @brief	ȫ�ֱ���_����ʱ��Ҫ��Ƶ�ʲ���	*/
extern int _global_diffractRayNum;														/** @brief	ȫ�ֱ���_��������ʱ��ɢ������	*/

#endif
