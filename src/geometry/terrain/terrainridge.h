#ifndef RTLBS_TERRAINRIDGE
#define RTLBS_TERRAINRIDGE

#include "rtlbs.h"
#include "utility/define.h"
#include "terrainprofilepoint.h"
#include "material/material.h"

//ɽ�Ͷ���
class TerrainRidge {
public:
	bool m_isValid;											/** @brief	ɽ���Ƿ���Ч��־λ	*/
	std::vector<TerrainProfilePoint*> m_points;				/** @brief	ɽ�������	*/
	TerrainProfilePoint* m_peak;							/** @brief	�嶥����	*/
	TerrainProfilePoint* m_leftValley;						/** @brief	����	*/
	TerrainProfilePoint* m_rightValley;						/** @brief	�ҷ��	*/
	RtLbsType m_curvRadius;									/** @brief	�嶥�뾶	*/
	RtLbsType m_actualHeight;								/** @brief	�嶥�߶�:��ʵ�߶�	*/
	RtLbsType m_relativeHeight;								/** @brief	�嶥�߶�:����շ������ϵĸ߶�	*/
	Material* m_mat;										/** @brief	����	*/

public:
	TerrainRidge();
	TerrainRidge(TerrainProfilePoint* peak, TerrainProfilePoint* leftValley, TerrainProfilePoint* rightValley, TerrainProfilePoint* sp, TerrainProfilePoint* ep, std::vector<TerrainProfilePoint*> points);
	TerrainRidge(const TerrainRidge& ridge);
	~TerrainRidge();
	TerrainRidge& operator = (const TerrainRidge& ridge);
	bool operator == (const TerrainRidge& other) const;
	bool operator != (const TerrainRidge& other) const;
	void Init(TerrainProfilePoint* peak, TerrainProfilePoint* leftValley, TerrainProfilePoint* rightValley, TerrainProfilePoint* sp, TerrainProfilePoint* ep, std::vector<TerrainProfilePoint*> points);
	RtLbsType GetNValue() const;												//������͵�Nֵ��UTD�������䣩
	void CalDiffractionParameters(const Vector3D& inDir, const Vector3D& diffDir, RtLbsType& incidentTheta, RtLbsType& diffractionTheta) const;		//����UTD����ǲ���
	void WriteToFile(std::ofstream& outFile) const;								//�����Ͷ���д�뵽�ļ���
private:
	RtLbsType _calCurvRadius(); //����ɽ�͵����ʰ뾶
	TerrainProfilePoint* _findPoint(int id) const; //Ѱ��Id��Ӧ�������,���Ҳ����򷵻�nullptr
	RtLbsType _calPeakRelativeHeightInTRX(TerrainProfilePoint* sp, TerrainProfilePoint* ep) const;			/** @brief	����嶥����TRX���ߵĴ��߾��룬�����ɸ�	*/
};

#endif
