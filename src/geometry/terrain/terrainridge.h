#ifndef RTLBS_TERRAINRIDGE
#define RTLBS_TERRAINRIDGE

#include "rtlbs.h"
#include "utility/define.h"
#include "terrainprofilepoint.h"
#include "material/material.h"

//山峦对象
class TerrainRidge {
public:
	bool m_isValid;											/** @brief	山峦是否有效标志位	*/
	std::vector<TerrainProfilePoint*> m_points;				/** @brief	山峦剖面点	*/
	TerrainProfilePoint* m_peak;							/** @brief	峰顶坐标	*/
	TerrainProfilePoint* m_leftValley;						/** @brief	左峰谷	*/
	TerrainProfilePoint* m_rightValley;						/** @brief	右峰谷	*/
	RtLbsType m_curvRadius;									/** @brief	峰顶半径	*/
	RtLbsType m_actualHeight;								/** @brief	峰顶高度:真实高度	*/
	RtLbsType m_relativeHeight;								/** @brief	峰顶高度:相对收发连线上的高度	*/
	Material* m_mat;										/** @brief	材料	*/

public:
	TerrainRidge();
	TerrainRidge(TerrainProfilePoint* peak, TerrainProfilePoint* leftValley, TerrainProfilePoint* rightValley, TerrainProfilePoint* sp, TerrainProfilePoint* ep, std::vector<TerrainProfilePoint*> points);
	TerrainRidge(const TerrainRidge& ridge);
	~TerrainRidge();
	TerrainRidge& operator = (const TerrainRidge& ridge);
	bool operator == (const TerrainRidge& other) const;
	bool operator != (const TerrainRidge& other) const;
	void Init(TerrainProfilePoint* peak, TerrainProfilePoint* leftValley, TerrainProfilePoint* rightValley, TerrainProfilePoint* sp, TerrainProfilePoint* ep, std::vector<TerrainProfilePoint*> points);
	RtLbsType GetNValue() const;												//计算峰峦的N值（UTD地形绕射）
	void CalDiffractionParameters(const Vector3D& inDir, const Vector3D& diffDir, RtLbsType& incidentTheta, RtLbsType& diffractionTheta) const;		//计算UTD绕射角参数
	void WriteToFile(std::ofstream& outFile) const;								//将峰峦对象写入到文件中
private:
	RtLbsType _calCurvRadius(); //计算山峦的曲率半径
	TerrainProfilePoint* _findPoint(int id) const; //寻找Id对应的坐标点,若找不到则返回nullptr
	RtLbsType _calPeakRelativeHeightInTRX(TerrainProfilePoint* sp, TerrainProfilePoint* ep) const;			/** @brief	计算峰顶距离TRX连线的垂线距离，可正可负	*/
};

#endif
