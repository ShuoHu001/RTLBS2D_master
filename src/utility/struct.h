#ifndef RTLBS_STRUCT
#define RTLBS_STRUCT

#include "rtlbs.h"
#include "define.h"

/**
* @brief		·��׷�ٹ����д�������ֹͣ��Ϣ
* @author		��˶
* @date			2024/04/14
*/
typedef struct {
	bool EndStopReflect;					/** @brief	������ֹ	*/
	bool EndStopTransmit;					/** @brief	͸����ֹ	*/
	bool EndStopDiffract;					/** @brief	������ֹ	*/
	bool EndStopScatter;					/** @brief	ɢ����ֹ	*/
}EndStopInfo;



#endif
