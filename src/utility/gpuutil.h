#ifndef RTLBS_GPUUTIL
#define RTLBS_GPUUTIL

#include "rtlbs.h"
#include "utility/define.h"

template <typename T>
void ClearDeviceVectorMemory(thrust::device_vector<T>& vec) {
	thrust::device_vector<T>().swap(vec);
}


#endif
