#ifndef RTLBS_SCENE
#define RTLBS_SCENE

#include "rtlbs.h"
#include "utility/define.h"
#include "geometry/bbox2d.h"
#include "geometry/segment2d.h"
#include "geometry/ray2d.h"
#include "geometry/Intersection2D.h"
#include "accel/accelerator.h"
#include "accel/bvh.h"
#include "accel/kdtree.h"
#include "accel/unigrid.h"
#include "accel/sdf.h"
#include "accel/uniformgrid.h"
#include "geometry/object2d/object2d.h"
#include "geometry/object2d/building2d.h"
#include "geometry/object2d/vegetation2d.h"
#include "geometry/object2d/wall2d.h"
#include "configuration/simconfig.h"
#include "configuration/geometryconfig.h"
#include "material/materiallibrary.h"
#include "equipment/antenna/antennalibrary.h"
#include "equipment/transceiver/transmitter.h"
#include "equipment/transceiver/receiver.h"
#include "radiowave/raypath/raytreenode.h"
#include "radiowave/raypath/terraindiffractionpath.h"
#include "radiowave/raypath/gpu/raypathgpu.h"
#include "equipment/sensor/sensor.h"
#include "equipment/sensor/sensordata.h"
#include "geometry/terrain/terrain.h"
#include "equipment/sensor/sensordatalibrary.h"
#include "radiowave/propagation/diffraction.h"
#include "radiowave/propagation/reflection.h"

class RayPath3D;

class Scene {
public:
	bool m_loadingTerrainFlag;									/** @brief	是否加载地形	*/
	MaterialLibrary m_materialLibrary;							/** @brief	场景中的材质库	*/
	AntennaLibrary m_antennaLibrary;							/** @brief	场景中的天线库	*/
	std::vector<Segment2D*> m_segmentBuf;						/** @brief	场景中的所有二维线段	*/
	std::vector<Segment2DGPU> m_gpuSegmentBuf;					/** @brief	场景中的GPU中所有的二维线段（存储在CPU内存中）	*/
	std::vector<Wedge2D*> m_wedgeBuf;							/** @brief	场景中的所有楔形角	*/
	std::vector<Wedge2DGPU> m_gpuWedgeBuf;						/** @brief	场景中的GPU中所有的楔形角（存储在GPU内存中）	*/
	std::vector<Object2D*> m_objects;							/** @brief	场景中的所有物体文件	*/
	Terrain m_terrain;											/** @brief	场景中的地形	*/
	Accelerator* m_pAccelerator;								/** @brief	场景中的加速结构	*/
	std::vector<Transmitter*> m_transmitters;					/** @brief	场景中的发射机	*/
	std::vector<Receiver*> m_receivers;							/** @brief	场景中的接收机	*/
	std::vector<Sensor*> m_sensors;								/** @brief	场景中的传感器	*/
	SensorDataLibrary m_sensorDataLibrary;						/** @brief	场景中的传感器数据库	*/
	std::string m_filename;
	mutable BBox2D m_bbox;										/** @brief	场景包围盒	*/

public:
	Scene();
	~Scene();
	bool LoadScene(const SimConfig& config);																				//基于几何配置初始化场景
	void ConvertToGPUHostScene();																							//将场景中的环境转换为GPU中（内存数据仍然在主机中）
	bool GetIntersect(const Ray2D& r, Intersection2D* intersect) const;															//求解射线与环境的相交情况
	bool GetIntersect(const Segment2D& segment, Intersection2D* intersect) const;											//求解线段与环境的相交情况
	bool IsBlock(const Point3D& ps, const Point3D& pe) const;																//判定两点组成的线段是否被环境遮挡
	bool IsBlockOnlyObject(const Point3D& ps, const Point3D& pe) const;														//判定两点组成的线段是否被环境遮挡-仅适用于环境中物体
	bool GetRayTubeWedges(const Ray2D& r,RayTreeNode* treenode, Intersection2D* intersect) const;							//求解在射线管内的所有可见的wedegs
	void Release();
	void OutputLog() const;
	void PreProcess(ACCEL_TYPE type);
	const BBox2D GetBBox() const;
	const std::string& GetFileName() const { return m_filename; }
	RtLbsType _getPositionRefractN(Point2D& p);																				//根据输入的坐标，获取环境中某点位置处的透射系数
	bool IsValidPoint(const Point3D& p) const;																				//检测三维坐标点是否有效
	bool IsValidPoint(const Point2D& p) const;																				//检测二维坐标点是否有效
	bool IsNearSegmentPoint(const Point2D& p, RtLbsType threshold) const;													//检测二维坐标点是否靠近墙体,阈值距离为threshold
	bool InitSceneTransmitters(const TransmitterCollectionConfig& config, AntennaLibrary* antLibrary);						//初始化场景中的发射机
	bool InitSceneReceivers(const std::vector<ReceiverUnitConfig>& configs, AntennaLibrary* antLibrary);					//初始化场景中的接收机
	bool InitSceneSensors(const SensorCollectionConfig& config, AntennaLibrary* antLibrary, bool hasSimuError);								//初始化场景中的传感器
	bool GetGroundReflectPaths(const Point3D& ps, const Point3D& pe, std::vector<RayPath3D*>& outPaths) const;				//获得地面反射路径
	bool GetGroundDiffractionPath(const Point3D& ps, const Point3D& pe, TerrainDiffractionPath*& outPath) const;			//获得地面绕射路径
	bool IsValidRayPath(const RayPath3D* path) const;																		//判定常规路径是否有效，利用地形进行判定
	bool IsValidRayPath(const TerrainDiffractionPath* path) const;															//判定地形绕射路径是否有效，利用常规物体进行判定


private:
	bool _bfIntersect(const Ray2D& r, Intersection2D* intersect) const;														//射线与环境元素进行相交判定-暴力求解
	bool _isRepeatToTransmitter(const Point3D& p) const;																	//检测坐标是否与发射机相同

};

#endif
