#ifndef RTLBS_TERRAINPROFILE
#define RTLBS_TERRAINPROFILE

#include "rtlbs.h"
#include "utility/define.h"
#include "terrainprofilepoint.h"
#include "terrainprofilesegment.h"
#include "terrainridge.h"
#include "tree/terraindiffractionpath.h"
#include "material/material.h"

//�����������
class TerrainProfile {
public:
	int m_validRidgeNum;							/** @brief	��Ч�ķ��͸���	*/
	RtLbsType m_minH;								/** @brief	�����������ֵ	*/
	RtLbsType m_maxH;								/** @brief	�����������ֵ	*/
	RtLbsType m_meanH;								/** @brief	ƽ���߶�(��ֵ)	*/
	RtLbsType m_stdH;								/** @brief	���θ߶�ƽ��ƫ��ֵ	*/
	RtLbsType m_undulation;							/** @brief	��������̶ȣ����ֵ����Сֵ��Ĳ�ࣩ	*/
private:
	std::vector<TerrainProfilePoint*> m_points;		/** @brief	���������	*/
	std::vector<TerrainRidge*> m_ridges;			/** @brief	ɽ��	*/
	std::vector<TerrainProfilePoint*> m_peaks;		/** @brief	ɽ��	*/
	std::vector<TerrainProfilePoint*> m_valley;		/** @brief	ɽ��	*/
	Point3D m_txPosition;							/** @brief	�����λ��	*/
	Point3D m_rxPosition;							/** @brief	���ջ�λ��	*/

public:
	TerrainProfile();
	~TerrainProfile();
	void InitParameters(const std::vector<Point3D>& points, const std::vector<Material*>& mats, const Point3D& txPosition, const Point3D& rxPosition, RtLbsType averRidgeGap);							//��ʼ���������
	void GetDiffractPathOverRidges(TerrainDiffractionPath*& outPath) const;
	void WritePeaksToFile(std::string filename) const;													//��peaksд���ļ���
	void WriteValleysToFile(std::string filename) const;												//��valleysд���ļ���
	void WriteRidgesToFile(std::string filename) const;													//��ridgesд�뵽�ļ���
	void WriteProfileToFile(std::string filename) const;												//����������д�뵽�ļ���

private:
	void _init(const std::vector<Point3D>& points, const std::vector<Material*>& mats);										//��ʼ����������(Я�����β�����Ϣ)
	void _calStaticalParameters();																						//����ͳ�Ʋ���
	bool _isValidPeak(const TerrainProfilePoint* p) const;																//�ж�peak�Ƿ���Ч
	bool _getIntersect(TerrainProfileSegment& segment);																	//�ж����������߶�����������Ƿ��ཻ
	void _findRidges();																									//Ѱ�ҵ��������е�ɽ��,��pΪ���
	TerrainProfilePoint* _findLeftValley(TerrainProfilePoint* peak) const;												//���ڸ����ķ�ֵ������������Сֵ��
	TerrainProfilePoint* _findRightValley(TerrainProfilePoint* peak) const;												//���ڸ����ķ�ֵ������������Сֵ��
	TerrainProfilePoint* _findValley(std::vector<TerrainProfilePoint*>& valleys, RtLbsType tmin, RtLbsType tmax);		//���������Сֵ���������ȵ����ݼ��е���Сֵ
	std::vector<TerrainProfilePoint*> _getPoints(int idMin, int idMax) const;											//Ѱ����С���������Ӧ�����������	
};

#endif
