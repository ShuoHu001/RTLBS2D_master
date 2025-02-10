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
	bool m_loadingTerrainFlag;									/** @brief	�Ƿ���ص���	*/
	MaterialLibrary m_materialLibrary;							/** @brief	�����еĲ��ʿ�	*/
	AntennaLibrary m_antennaLibrary;							/** @brief	�����е����߿�	*/
	std::vector<Segment2D*> m_segmentBuf;						/** @brief	�����е����ж�ά�߶�	*/
	std::vector<Segment2DGPU> m_gpuSegmentBuf;					/** @brief	�����е�GPU�����еĶ�ά�߶Σ��洢��CPU�ڴ��У�	*/
	std::vector<Wedge2D*> m_wedgeBuf;							/** @brief	�����е�����Ш�ν�	*/
	std::vector<Wedge2DGPU> m_gpuWedgeBuf;						/** @brief	�����е�GPU�����е�Ш�νǣ��洢��GPU�ڴ��У�	*/
	std::vector<Object2D*> m_objects;							/** @brief	�����е����������ļ�	*/
	Terrain m_terrain;											/** @brief	�����еĵ���	*/
	Accelerator* m_pAccelerator;								/** @brief	�����еļ��ٽṹ	*/
	std::vector<Transmitter*> m_transmitters;					/** @brief	�����еķ����	*/
	std::vector<Receiver*> m_receivers;							/** @brief	�����еĽ��ջ�	*/
	std::vector<Sensor*> m_sensors;								/** @brief	�����еĴ�����	*/
	SensorDataLibrary m_sensorDataLibrary;						/** @brief	�����еĴ��������ݿ�	*/
	std::string m_filename;
	mutable BBox2D m_bbox;										/** @brief	������Χ��	*/

public:
	Scene();
	~Scene();
	bool LoadScene(const SimConfig& config);																				//���ڼ������ó�ʼ������
	void ConvertToGPUHostScene();																							//�������еĻ���ת��ΪGPU�У��ڴ�������Ȼ�������У�
	bool GetIntersect(const Ray2D& r, Intersection2D* intersect) const;															//��������뻷�����ཻ���
	bool GetIntersect(const Segment2D& segment, Intersection2D* intersect) const;											//����߶��뻷�����ཻ���
	bool IsBlock(const Point3D& ps, const Point3D& pe) const;																//�ж�������ɵ��߶��Ƿ񱻻����ڵ�
	bool IsBlockOnlyObject(const Point3D& ps, const Point3D& pe) const;														//�ж�������ɵ��߶��Ƿ񱻻����ڵ�-�������ڻ���������
	bool GetRayTubeWedges(const Ray2D& r,RayTreeNode* treenode, Intersection2D* intersect) const;							//��������߹��ڵ����пɼ���wedegs
	void Release();
	void OutputLog() const;
	void PreProcess(ACCEL_TYPE type);
	const BBox2D GetBBox() const;
	const std::string& GetFileName() const { return m_filename; }
	RtLbsType _getPositionRefractN(Point2D& p);																				//������������꣬��ȡ������ĳ��λ�ô���͸��ϵ��
	bool IsValidPoint(const Point3D& p) const;																				//�����ά������Ƿ���Ч
	bool IsValidPoint(const Point2D& p) const;																				//����ά������Ƿ���Ч
	bool IsNearSegmentPoint(const Point2D& p, RtLbsType threshold) const;													//����ά������Ƿ񿿽�ǽ��,��ֵ����Ϊthreshold
	bool InitSceneTransmitters(const TransmitterCollectionConfig& config, AntennaLibrary* antLibrary);						//��ʼ�������еķ����
	bool InitSceneReceivers(const std::vector<ReceiverUnitConfig>& configs, AntennaLibrary* antLibrary);					//��ʼ�������еĽ��ջ�
	bool InitSceneSensors(const SensorCollectionConfig& config, AntennaLibrary* antLibrary, bool hasSimuError);								//��ʼ�������еĴ�����
	bool GetGroundReflectPaths(const Point3D& ps, const Point3D& pe, std::vector<RayPath3D*>& outPaths) const;				//��õ��淴��·��
	bool GetGroundDiffractionPath(const Point3D& ps, const Point3D& pe, TerrainDiffractionPath*& outPath) const;			//��õ�������·��
	bool IsValidRayPath(const RayPath3D* path) const;																		//�ж�����·���Ƿ���Ч�����õ��ν����ж�
	bool IsValidRayPath(const TerrainDiffractionPath* path) const;															//�ж���������·���Ƿ���Ч�����ó�����������ж�


private:
	bool _bfIntersect(const Ray2D& r, Intersection2D* intersect) const;														//�����뻷��Ԫ�ؽ����ཻ�ж�-�������
	bool _isRepeatToTransmitter(const Point3D& p) const;																	//��������Ƿ��뷢�����ͬ

};

#endif
