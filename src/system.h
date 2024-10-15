#ifndef RTLBS_SYSTEM
#define RTLBS_SYSTEM

#include "rtlbs.h"
#include "utility/define.h"
#include "scene/scene.h"
#include "configuration/simconfig.h"
#include "result/result.h"
#include "radiowave/raypath/pathnode.h"
#include "radiowave/raypath/raytreenode.h"
#include "managers/logmanager.h"
#include "math/complex.h"
#include "data/tranfunctiondata.h"
#include "configuration/frequencyconfig.h"
#include "material/materiallibrary.h"
#include "utility/enum.h"
#include "core/render/pathtracing/raylaunch.h"
#include "core/render/pathtracing/raytracing.h"
#include "core/render/pathtracing/pathbuilder.h"
#include "core/render/pathtracing/treenodegenerator.h"
#include "core/render/pathtracing/gpu/pathtracinggpu.h" //���м���RT��
#include "core/render/pathtracing/pathtracing.h"
#include "radiowave/raypath/gpu/treenodegpu.h"
#include "equipment/sensor/sensordata.h"
#include "localization/lbsinfocluster.h"
#include "localization/aoa/aoa_locator.h"
#include "localization/tdoa/tdoa_locator.h"
#include "general/elevationmatrix.h"


class System {

public:
	SYSTEM_MODE m_sysMode;														/** @brief	ϵͳ����ģʽ	*/
	Scene* m_scene;																/** @brief	��������	*/
	SimConfig m_simConfig;														/** @brief	���������ļ�	*/
	Result m_result;															/** @brief	�������ļ�	*/
	LBSInfoCluster m_lbsInfoCluster;											/** @brief	��λ��Ϣ����	*/

private:
	std::string m_configFileName;												/** @brief	�����ļ�����	*/

	std::vector<RayTreeNode*> m_rtTreeRoot;										/** @brief	�������ṹ���飬raytracingģʽ�õ�	*/
	std::vector<std::vector<Ray2D>> m_rtInitRays;								/** @brief	��ʼ�����߼���, 1D:tx,2D:raynum*/

	std::vector<RayTreeNode*> m_lbsTreeRoot;									/** @brief	����׷�ٽṹ���飬LBSģʽ�õ�	*/
	std::vector<std::vector<Ray2D>> m_lbsInitRays;								/** @brief	��ʼ�����߼��ϣ�LBSģʽ�õ�	*/

	std::vector<std::vector<PathNodeGPU*>> m_rtGPUPathNodes;					/** @brief	GPU�����������·�����,��һ��Ϊtx�������ڶ���Ϊÿ��rx�е�����	*/
	std::vector<std::vector<TreeNodeGPU*>> m_gpuTreeNodes;						/** @brief	GPU��������������ڵ����ݣ���һ��Ϊsensor�������ڶ���Ϊÿ��sensor�е�����	*/

	std::vector<Complex> m_tranFunctionData;									/** @brief	��������õĴ��亯��	*/

	ElevationMatrix m_lbsShiftErrorMatrix;										/** @brief	��λ������߶�λ���ȵ�λ�Ʊ仯������	*/

public:
	System();
	~System();

public:
	bool Setup(SYSTEM_MODE mode = MODE_RT);										//����ϵͳ
	void Render();																//��������׷�ٽ��е����Ⱦ
	void RayLaunch(const RAYLAUNCHMODE& mode, uint64_t rayNum);					//���߷���ģ��
	void RayTracing(const HARDWAREMODE mode);									//����׷��ģ��
	void RayTracingLBS(const HARDWAREMODE mode);								//��λ�㷨�е�����׷��ģ��
	void PathBuilder(const HARDWAREMODE mode);									//·������ģ��
	void TreeNodeGenerator(const HARDWAREMODE hardwareNode);					//���ڵ����ģ��,���ڲ�������Դ
	void PostProcessing();														//�������ݣ��㷨��أ�
	Point2D TargetLocalization(LOCALIZATION_MODE lbsMode, LOCALIZATION_METHOD lbsMethod);			//��λ�㷨
	void OutputResults();														//�������
	void PreProcess();															//Ԥ����ϵͳ
	

private:
	bool _initRTSystemMemory();													//��ʼ������׷��ϵͳ�ڴ�
	bool _initLBSSystemMemory();												//��ʼ����λϵͳ�ڴ�
};


#endif
