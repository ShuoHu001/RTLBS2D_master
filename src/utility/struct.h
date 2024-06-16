#ifndef RTLBS_STRUCT
#define RTLBS_STRUCT

#include "rtlbs.h"
#include "define.h"

/**
* @brief		路径追踪过程中传播机制停止信息
* @author		胡硕
* @date			2024/04/14
*/
typedef struct {
	bool EndStopReflect;					/** @brief	反射终止	*/
	bool EndStopTransmit;					/** @brief	透射终止	*/
	bool EndStopDiffract;					/** @brief	绕射终止	*/
	bool EndStopScatter;					/** @brief	散射终止	*/
}EndStopInfo;



#endif
