#include "globalvariables.h"
#include "antenna/antenna.h"

std::vector<Complex> _global_tranFunctionData;							/** @brief	ȫ�ֱ���_��������ϵ�����õĴ��亯�������������֣�	*/
std::vector<Antenna> _global_innerAntennas;								/** @brief	ȫ�ֱ���_����ʱ��Ҫ���������߷���ͼ	*/
FrequencyConfig _global_freqConfig;										/** @brief	ȫ�ֱ���_����ʱ��Ҫ��Ƶ�ʲ���	*/
int _global_diffractRayNum;												/** @brief	ȫ�ֱ���_��������ʱ��ɢ������	*/
ElevationMatrix _global_lbsShiftErrorMatrix;							/** @brief	ȫ�ֱ���_��λ������߶�λ���ȵ�λ�Ʊ仯������	*/