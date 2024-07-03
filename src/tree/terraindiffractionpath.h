#ifndef RTLBS_TERRAINDIFFRACTIONPATH
#define RTLBS_TERRAINDIFFRACTIONPATH

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point3d.h"
#include "geometry/terrain/terrainridge.h"
#include "raypath.h"
#include "antenna/antenna.h"
#include "material/materiallibrary.h"
#include "physical/radiowave/polarization3d.h"



//��ͷ�ļ�Ϊ��������·����������


//����·���ڵ�����
class TerrainPathNode {
public:
	Point3D m_point;					/** @brief	������	*/
	RtLbsType m_clearanceHeight;		/** @brief	���ո߶�	*/
	RtLbsType m_s1;						/** @brief	�������ĸ߶�	*/
	RtLbsType m_s2;						/** @brief	�����յ�ĸ߶�	*/
	const TerrainRidge* m_ridge;		/** @brief	ÿ���ڵ��Ӧ�ķ���	*/

public:
	TerrainPathNode();
	TerrainPathNode(const Point3D& edgePoint);																		//��ʼ��-�շ��ڵ㣨�Ƿ�ֵ�ڵ㣩
	TerrainPathNode(const TerrainRidge* ridge);
	TerrainPathNode(Point3D& p, RtLbsType& clearanceHeight, RtLbsType& sPs, RtLbsType& sPe);
	TerrainPathNode(const TerrainRidge* prevRidge, const TerrainRidge* curRidge, const TerrainRidge* nextRidge);
	TerrainPathNode(const TerrainPathNode& node);
	~TerrainPathNode();
	TerrainPathNode& operator = (const TerrainPathNode& node);
	bool operator == (const TerrainPathNode& node) const;
	bool operator != (const TerrainPathNode& node) const;
};


//��������·��
class TerrainDiffractionPath {
public:
	std::vector<TerrainPathNode*> m_nodes;				/** @brief	�������������	*/
	TERRAINDIFFRACTIONMODE m_terrainDiffractionMode;	/** @brief	�����������ģʽ	*/

public:
	TerrainDiffractionPath();
	~TerrainDiffractionPath();
	Complex CalculateDiffractionEField_PICQUENARD(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;			//�������������� Picquenard ����
	Complex CalculateDiffractionEField_EPSTEIN(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;				//�������������� EPSTEIN����
	Complex CalculateDiffractionEField_UTD(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;					//�������������� UTD����
	Complex CalculateTerrainDiffractionEField(RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna) const;										//�����������
	RtLbsType GetPropagationTime() const;							//����·������ʱ��
	RtLbsType GetPropagationLength() const;							//����·����������
	RtLbsType GetPhaseOffset(RtLbsType freq) const;					//������λƫ��
	RtLbsType GetAngleofDeparture_Phi() const;						//����·���뿪��λ��
	RtLbsType GetAngleofDeparture_Theta() const;					//����·���뿪������
	RtLbsType GetAngleofArrival_Phi() const;						//����·�����︶������
	RtLbsType GetAngleofArrival_Theta() const;						//����·�����︩����
	RtLbsType CalDopplerShift(RtLbsType freq, const Vector3D& txVelocity, const Vector3D& rxVelocity);		//�������·���Ķ�����Ƶ��
	void OuputRaypath(std::ofstream& stream) const;					//����ྶ��Ϣ
	void RectifySParameters();										//����S������Ϣ
};


#endif
