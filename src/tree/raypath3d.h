#ifndef RTLBS_RAYPATH3D
#define RTLBS_RAYPATH3D

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point3d.h"
#include "geometry/vector3d.h"
#include "physical/radiowave/polarization3d.h"
#include "material/materiallibrary.h"
#include "material/material.h"
#include "antenna/antenna.h"
#include "raypath.h"
#include "pathnode3d.h"

class RayPathGPU;

class RayPath3D {
public:
	std::vector<PathNode3D*> m_nodes;						/** @brief	·���ڵ㼯��	*/
	RtLbsType m_propagationLength;							/** @brief	��������	*/
	RtLbsType m_energyRatio;								/** @brief	��������	*/
	bool m_containRefract;									/** @brief	�Ƿ����͸��	*/
	bool m_containSplitNode;								/** @brief	�Ƿ�������ѽڵ�	*/
	bool m_isValid;											/** @brief	�Ƿ�����Ч·��	*/
	RAYPATHTYPE m_type;										/** @brief	·������	*/
	int m_angularSpectrumCategoryId;						/** @brief	·�����ڵĴ������Ƕ�������ID	*/

public:
	RayPath3D();																									//Ĭ�Ϲ��캯��
	RayPath3D(const RayPath& path, const Point3D& tx, const Point3D& rx);											//��ά����·�����캯��
	RayPath3D(const RayPath& path, RtLbsType height);																//��ά����·�����캯��-���ڶ�λ�еĵ�ż���
	RayPath3D(const RayPathGPU& path, const Point3D& tx, const Point3D& rx, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges);			//GPU��ά����·�����캯��
	RayPath3D(const RayPath3D& path);																				//��ֵ���캯��
	~RayPath3D();																									//��������
	RayPath3D& operator = (const RayPath3D& path);																	//��ֵ������
	void ReverseRayPath();																							//��·����ͷ��β��������ת·��

	void ConvertByRayPath(const RayPath& path, const Point3D& tx, const Point3D& rx);								//����ά·��ת��Ϊ��ά·��
	void Union(PathNode3D* node);																					//��չ·���ڵ�
	Polarization3D CalculateStrengthField3D(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna);																					//����·����ά���糡ֵ���漰���亯���ļ��㣩
	Polarization3D CalculateStrengthField3D(RtLbsType power, RtLbsType freq,
		const MaterialLibrary* matLibrary, const Antenna* txAntenna);												//����·����ά���糡ֵ�����漰���亯���ļ��㣩
	Polarization3D CalculateStrengthField3DReverse(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna);																					//�������·����ά���糡ֵ���漰���亯���ļ��㣩
	Polarization3D CalculateStrengthField3DReverse(RtLbsType power, RtLbsType freq,
		const MaterialLibrary* matLibrary, const Antenna* txAntenna);												//�������·����ά���糡ֵ�����漰���亯���ļ��㣩
	Complex CalculateStrengthField(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//����·�����糡ֵ���漰���亯���ļ��㣩
	Complex CalculateStrengthField(RtLbsType power, RtLbsType freq, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//����·�����糡ֵ�����漰���亯���ļ��㣩
	Complex CalculateStrengthFieldReverse(RtLbsType power, RtLbsType freq,
		const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//�������·�����糡ֵ���漰���亯���ļ��㣩
	Complex CalculateStrengthFieldReverse(RtLbsType power, RtLbsType freq, const MaterialLibrary* matLibrary,
		const Antenna* txAntenna, const Antenna* rxAntenna);														//�������·�����糡ֵ�����漰���亯���ļ��㣩
	RtLbsType CalculatePowerInLBSSystem(RtLbsType freq, const std::vector<Complex>& tranFucntion, const MaterialLibrary* matLibrary, const Antenna* trxAntenna);			//����LBS��λϵͳ�еĹ���ֵ
	RtLbsType GetPropagationTime() const;																			//����ྶ����ʱ��
	RtLbsType GetPropagationLength() const;																			//���㴫������
	RtLbsType GetPhaseOffset(RtLbsType freq) const;																	//������λƫ����	
	RtLbsType GetAOD_Phi() const;																					//����·���뿪��λ��
	RtLbsType GetAOD_Theta() const;																					//����·���뿪������
	RtLbsType GetAOA_Phi() const;																					//����·�����︶������
	RtLbsType GetAOA_Theta() const;																					//����·�����︩����
	void DeepCopy(const RayPath3D* path);																				//��ȸ���
	void DeepDestroy();																								//�������
	void Clear();
	void OutputRaypath(std::ofstream& stream) const;																//����ྶ���ļ���
};

#endif
