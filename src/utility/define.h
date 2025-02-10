#ifndef RTLBS_DEFINE
#define RTLBS_DEFINE

#define GLOG_NO_ABBREVIATED_SEVERITIES

#define RTLBS_DOUBLE_DECLARATION  //系统使用double声明

#ifndef RTLBS_DOUBLE_DECLARATION
typedef float RtLbsType;
#define EPSILON 1e-5 //十万分之一精度
#else
typedef double RtLbsType;
#define EPSILON 1e-6 //十万分之一精度
#endif

#define TRAN_EPSILON 1e-2 //透射修正极限为1cm


#define	PI				3.14159265358979323846			//pi
#define	TWO_PI			6.28318530717958647692			//2*pi
#define THREE_PI		9.42477796076937971538			//3*pi
#define FOUR_PI			12.5663706143591729538			//4*pi
#define HALF_PI			1.57079632679489661923			//二分之一倍的pi
#define THIRD_PI		1.04719755119659774615			//三分之一倍的pi
#define QUARTER_PI		0.78539816339744830962			//四分之一倍的pi
#define	INV_PI			0.31830988618379067153			//pi分之一
#define INV_TWO_PI		0.15915494309189533576			//2pi分之一
#define INV_THREE_PI	0.10610329539459689051			//3pi分之一
#define INV_FOUR_PI		0.07957747154594766788			//4pi分之一
#define TWO_THIRD_PI	2.09439510239319549230			//120度，三分之二pi
#define ONE_DEGEREE		0.01745329251994329577			//1度对应的角度值
#define TWO_DEGEREE		0.03490658503988659154			//2度对应的角度值
#define THREE_DEGREE	0.05235987755982988731			//3度对应的角度值
#define FOUR_DEGREE		0.06981317007977318308			//4度对应的角度值
#define FIVE_DEGREE		0.08726646259971647885			//5度对应的角度值


#define SQRT2			1.41421356237309504880			//sqrt(2)
#define TWO_SQRT2		2.82842712474619009760			//2*sqrt(2)
#define THREE_SQRT2		4.24264068711928514641			//3*sqrt(2)
#define FOUR_SQRT2		5.65685424949238019521			//4*sqrt(2)
#define HALF_SQRT2		0.70710678118654752440			//0.5*sqrt(2)
#define THIRD_SQRT2		0.47140452079103168293			//三分之一倍的sqrt(2)
#define QUARTER_SQRT2	0.35355339059327376220			//四分之一倍的sqrt(2)

#define SQRT3			1.73205080756887729353			//sqrt(3)
#define TWO_SQRT3		3.46410161513775458705			//2*sqrt(3)
#define THREE_SQRT3		5.19615242270663188058			//3*sqrt(3)
#define FOUR_SQRT3		6.92820323027550917411			//4*sqrt(3)
#define HALF_SQRT3		0.86602540378443864676			//二分之一倍的sqrt(3)
#define THIRD_SART3		0.57735026918962576451			//三分之一倍的sqrt(3)
#define QUARTER_SQRT3	0.43301270189221932338			//四份之一倍的sqrt(3)

#define SQRT5			2.23606797749978969641			//sqrt(5)
#define TWO_SQRT5		4.47213595499957939282			//2*sqrt(5)
#define THREE_SQRT5		6.70820393249936908923			//3*sqrt(5)
#define FOUR_SQRT5		8.94427190999915878564			//4*sqrt(5)
#define HALF_SQRT5		1.11803398874989484820			//二分之一倍的sqrt(5)
#define THIRD_SQRT5		0.74535599249992989880			//三分之一倍的sqrt(5)
#define QUARTER_SQRT5	0.55901699437494742410			//四分之一倍的sqrt(5)

#define ONE_THIRD		0.33333333333333333333			//三分之一

#define LIGHT_VELOCITY_AIR  299702547.1					//光在空气中的传播速度(标准大气压，20℃)
#define LIGHT_DISTANCE_PER_NS 0.2997025471				//光在空气中一纳秒的传播距离，单位m
#define LIGHT_VELOCITY_VACUME  299792458				//光在真空中的传播速度
#define VACUME_PERMITTIVITY 8.8541e-12					//真空中的相对介电常数


#define EERC_K			1.33333333333333333333			//等效地球半径系数 (Equivalent Earth radius coefficient), Epstein 绕射系数用
#define EERC_K_INV		0.75							//等效地球半径系数的倒数


//GPU并行计算相关
#define SYSTEM_GPU_PARALLEL_MODE

#ifndef SYSTEM_GPU_PARALLEL_MODE
#define GPU_PARALLEL
#else
#define CPU_PARALLEL
#endif


#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/swap.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/remove.h>
#include <thrust/reduce.h>

#ifndef __CUDACC_RTC__  
#define __CUDACC_RTC__
#endif


#ifdef __INTELLISENSE__
#define CUDA_KERNEL(...)
#else
#define CUDA_KERNEL(...) <<< __VA_ARGS__ >>>
#endif

#define HOST_FUNC __host__
#define DEVICE_FUNC __device__
#define HOST_DEVICE_FUNC __host__ __device__
#define GLOBAL_FUNC __global__


//diffraction control
#define DIFF_DELTARAYNUM static_cast<unsigned>(50) //绕射射线数量浮动最大值

//ray-split control
#define RAYSPLITSTATE false      //射线分裂开关

//control console color
#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_RESET   "\x1b[0m"

// some useful macro
#define SAFE_DELETE(p) { if(p) { delete p; p = 0; } }
#define SAFE_DELETE_ARRAY(p) { if(p) { delete[] p; p = 0 ; } }

#define WEDGECRITICALANGLE 10/180*PI //绕射棱劈极限判定角度：10°


// math macros
#define saturate(x) max(0.0f,min(1.0f,x))

inline float clamp(float x, float mi, float ma)
{
	if (x > ma) x = ma;
	if (x < mi) x = mi;
	return x;
}
#endif
