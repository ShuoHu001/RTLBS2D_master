#include "globalvariables.h"
#include "antenna/antenna.h"

std::vector<Complex> _global_tranFunctionData;							/** @brief	ȫ�ֱ���_��������ϵ�����õĴ��亯�������������֣�	*/
std::vector<Antenna> _global_innerAntennas;								/** @brief	ȫ�ֱ���_����ʱ��Ҫ���������߷���ͼ	*/
FrequencyConfig _global_freqConfig;