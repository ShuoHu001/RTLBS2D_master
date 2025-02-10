#ifndef RTLBS_DEFINE
#define RTLBS_DEFINE

#define GLOG_NO_ABBREVIATED_SEVERITIES

#define RTLBS_DOUBLE_DECLARATION  //ϵͳʹ��double����

#ifndef RTLBS_DOUBLE_DECLARATION
typedef float RtLbsType;
#define EPSILON 1e-5 //ʮ���֮һ����
#else
typedef double RtLbsType;
#define EPSILON 1e-6 //ʮ���֮һ����
#endif

#define TRAN_EPSILON 1e-2 //͸����������Ϊ1cm


#define	PI				3.14159265358979323846			//pi
#define	TWO_PI			6.28318530717958647692			//2*pi
#define THREE_PI		9.42477796076937971538			//3*pi
#define FOUR_PI			12.5663706143591729538			//4*pi
#define HALF_PI			1.57079632679489661923			//����֮һ����pi
#define THIRD_PI		1.04719755119659774615			//����֮һ����pi
#define QUARTER_PI		0.78539816339744830962			//�ķ�֮һ����pi
#define	INV_PI			0.31830988618379067153			//pi��֮һ
#define INV_TWO_PI		0.15915494309189533576			//2pi��֮һ
#define INV_THREE_PI	0.10610329539459689051			//3pi��֮һ
#define INV_FOUR_PI		0.07957747154594766788			//4pi��֮һ
#define TWO_THIRD_PI	2.09439510239319549230			//120�ȣ�����֮��pi
#define ONE_DEGEREE		0.01745329251994329577			//1�ȶ�Ӧ�ĽǶ�ֵ
#define TWO_DEGEREE		0.03490658503988659154			//2�ȶ�Ӧ�ĽǶ�ֵ
#define THREE_DEGREE	0.05235987755982988731			//3�ȶ�Ӧ�ĽǶ�ֵ
#define FOUR_DEGREE		0.06981317007977318308			//4�ȶ�Ӧ�ĽǶ�ֵ
#define FIVE_DEGREE		0.08726646259971647885			//5�ȶ�Ӧ�ĽǶ�ֵ


#define SQRT2			1.41421356237309504880			//sqrt(2)
#define TWO_SQRT2		2.82842712474619009760			//2*sqrt(2)
#define THREE_SQRT2		4.24264068711928514641			//3*sqrt(2)
#define FOUR_SQRT2		5.65685424949238019521			//4*sqrt(2)
#define HALF_SQRT2		0.70710678118654752440			//0.5*sqrt(2)
#define THIRD_SQRT2		0.47140452079103168293			//����֮һ����sqrt(2)
#define QUARTER_SQRT2	0.35355339059327376220			//�ķ�֮һ����sqrt(2)

#define SQRT3			1.73205080756887729353			//sqrt(3)
#define TWO_SQRT3		3.46410161513775458705			//2*sqrt(3)
#define THREE_SQRT3		5.19615242270663188058			//3*sqrt(3)
#define FOUR_SQRT3		6.92820323027550917411			//4*sqrt(3)
#define HALF_SQRT3		0.86602540378443864676			//����֮һ����sqrt(3)
#define THIRD_SART3		0.57735026918962576451			//����֮һ����sqrt(3)
#define QUARTER_SQRT3	0.43301270189221932338			//�ķ�֮һ����sqrt(3)

#define SQRT5			2.23606797749978969641			//sqrt(5)
#define TWO_SQRT5		4.47213595499957939282			//2*sqrt(5)
#define THREE_SQRT5		6.70820393249936908923			//3*sqrt(5)
#define FOUR_SQRT5		8.94427190999915878564			//4*sqrt(5)
#define HALF_SQRT5		1.11803398874989484820			//����֮һ����sqrt(5)
#define THIRD_SQRT5		0.74535599249992989880			//����֮һ����sqrt(5)
#define QUARTER_SQRT5	0.55901699437494742410			//�ķ�֮һ����sqrt(5)

#define ONE_THIRD		0.33333333333333333333			//����֮һ

#define LIGHT_VELOCITY_AIR  299702547.1					//���ڿ����еĴ����ٶ�(��׼����ѹ��20��)
#define LIGHT_DISTANCE_PER_NS 0.2997025471				//���ڿ�����һ����Ĵ������룬��λm
#define LIGHT_VELOCITY_VACUME  299792458				//��������еĴ����ٶ�
#define VACUME_PERMITTIVITY 8.8541e-12					//����е���Խ�糣��


#define EERC_K			1.33333333333333333333			//��Ч����뾶ϵ�� (Equivalent Earth radius coefficient), Epstein ����ϵ����
#define EERC_K_INV		0.75							//��Ч����뾶ϵ���ĵ���


//GPU���м������
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
#define DIFF_DELTARAYNUM static_cast<unsigned>(50) //�������������������ֵ

//ray-split control
#define RAYSPLITSTATE false      //���߷��ѿ���

//control console color
#define COLOR_RED     "\x1b[31m"
#define COLOR_GREEN   "\x1b[32m"
#define COLOR_YELLOW  "\x1b[33m"
#define COLOR_RESET   "\x1b[0m"

// some useful macro
#define SAFE_DELETE(p) { if(p) { delete p; p = 0; } }
#define SAFE_DELETE_ARRAY(p) { if(p) { delete[] p; p = 0 ; } }

#define WEDGECRITICALANGLE 10/180*PI //�������������ж��Ƕȣ�10��


// math macros
#define saturate(x) max(0.0f,min(1.0f,x))

inline float clamp(float x, float mi, float ma)
{
	if (x > ma) x = ma;
	if (x < mi) x = mi;
	return x;
}
#endif
