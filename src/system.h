#ifndef RTLBS_SYSTEM
#define RTLBS_SYSTEM

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/scene.h"
#include "configuration/simconfig.h"
#include "result/result.h"
#include "tree/pathnode.h"
#include "tree/raytreenode.h"
#include "physical/pathtracing.h"
#include "parallel/threadpool.h"
#include "physical/gpu/pathtracinggpu.h" //并行计算RT库
#include "managers/logmanager.h"
#include "math/complex.h"
#include "data/tranfunctiondata.h"
#include "configuration/frequencyconfig.h"
#include "material/materiallibrary.h"
#include "utility/enum.h"
#include "physical/raylaunch.h"
#include "physical/raytracing.h"
#include "physical/pathbuilder.h"
#include "physical/treenodegenerator.h"
#include "tree/gpu/treenodegpu.h"
#include "equipment/sensordata.h"




class System {

public:
	SYSTEM_MODE m_sysMode;														/** @brief	系统工作模式	*/
	Scene* m_scene;																/** @brief	场景数据	*/
	SimConfig m_simConfig;														/** @brief	仿真配置文件	*/
	Result m_result;															/** @brief	仿真结果文件	*/
	

private:
	std::string m_configFileName;												/** @brief	配置文件名称	*/

	std::vector<RayTreeNode*> m_rtTreeRoot;										/** @brief	射线树结构数组，raytracing模式用到	*/
	std::vector<std::vector<Ray2D>> m_rtInitRays;								/** @brief	初始的射线集合, 1D:tx,2D:raynum*/

	std::vector<RayTreeNode*> m_lbsTreeRoot;									/** @brief	射线追踪结构数组，LBS模式用到	*/
	std::vector<std::vector<Ray2D>> m_lbsInitRays;								/** @brief	初始的射线集合，LBS模式用到	*/

	std::vector<Complex> m_tranFunctionData;									/** @brief	绕射计算过程中需要用到的过渡函数	*/

	std::vector<std::vector<PathNodeGPU*>> m_rtGPUPathNodes;					/** @brief	GPU加速中输出的路径结果,第一层为tx数量，第二层为每个rx中的数据	*/
	std::vector<std::vector<TreeNodeGPU*>> m_gpuTreeNodes;						/** @brief	GPU加速中输出的树节点数据，第一层为sensor数量，第二层为每个sensor中的数据	*/

public:
	System();
	~System();

public:
	bool Setup(SYSTEM_MODE mode = MODE_RT);										//设置系统
	void Render();																//基于射线追踪进行电磁渲染
	void RayLaunch(const RAYLAUNCHMODE& mode, uint64_t rayNum);					//射线发射模块
	void RayTracing(const HARDWAREMODE mode);									//射线追踪模块
	void RayTracingLBS(const HARDWAREMODE mode);								//定位算法中的射线追踪模块
	void PathBuilder(const HARDWAREMODE mode);									//路径构造模块
	void TreeNodeGenerator(const HARDWAREMODE hardwareNode);					//树节点产生模块
	void PostProcessing();														//后处理数据（算法相关）
	Point2D TargetLocalization(LOCALIZATION_MODE lbsMode, LOCALIZATION_METHOD lbsMethod);			//定位算法
	void OutputResults();														//输出数据
	void PreProcess();															//预处理系统
	

private:
	bool _initRTSystemMemory();													//初始化射线追踪系统内存
	bool _initLBSSystemMemory();												//初始化定位系统内存
};


#endif
