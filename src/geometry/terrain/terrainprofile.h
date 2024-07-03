#ifndef RTLBS_TERRAINPROFILE
#define RTLBS_TERRAINPROFILE

#include "rtlbs.h"
#include "utility/define.h"
#include "terrainprofilepoint.h"
#include "terrainprofilesegment.h"
#include "terrainridge.h"
#include "tree/terraindiffractionpath.h"
#include "material/material.h"

//地形剖面对象
class TerrainProfile {
public:
	int m_validRidgeNum;							/** @brief	有效的峰峦个数	*/
	RtLbsType m_minH;								/** @brief	地形剖面最低值	*/
	RtLbsType m_maxH;								/** @brief	地形剖面最高值	*/
	RtLbsType m_meanH;								/** @brief	平均高度(均值)	*/
	RtLbsType m_stdH;								/** @brief	地形高度平均偏离值	*/
	RtLbsType m_undulation;							/** @brief	地形起伏程度（最大值和最小值间的差距）	*/
private:
	std::vector<TerrainProfilePoint*> m_points;		/** @brief	地形剖面点	*/
	std::vector<TerrainRidge*> m_ridges;			/** @brief	山峦	*/
	std::vector<TerrainProfilePoint*> m_peaks;		/** @brief	山峰	*/
	std::vector<TerrainProfilePoint*> m_valley;		/** @brief	山谷	*/
	Point3D m_txPosition;							/** @brief	发射机位置	*/
	Point3D m_rxPosition;							/** @brief	接收机位置	*/

public:
	TerrainProfile();
	~TerrainProfile();
	void InitParameters(const std::vector<Point3D>& points, const std::vector<Material*>& mats, const Point3D& txPosition, const Point3D& rxPosition, RtLbsType averRidgeGap);							//初始化常规参数
	void GetDiffractPathOverRidges(TerrainDiffractionPath*& outPath) const;
	void WritePeaksToFile(std::string filename) const;													//将peaks写入文件中
	void WriteValleysToFile(std::string filename) const;												//将valleys写入文件中
	void WriteRidgesToFile(std::string filename) const;													//将ridges写入到文件中
	void WriteProfileToFile(std::string filename) const;												//将地形剖面写入到文件中

private:
	void _init(const std::vector<Point3D>& points, const std::vector<Material*>& mats);										//初始化地形剖面(携带地形材质信息)
	void _calStaticalParameters();																						//计算统计参数
	bool _isValidPeak(const TerrainProfilePoint* p) const;																//判定peak是否有效
	bool _getIntersect(TerrainProfileSegment& segment);																	//判定地形剖面线段与地形剖面是否相交
	void _findRidges();																									//寻找地形剖面中的山峦,以p为起点
	TerrainProfilePoint* _findLeftValley(TerrainProfilePoint* peak) const;												//基于给定的峰值点搜索左翼最小值点
	TerrainProfilePoint* _findRightValley(TerrainProfilePoint* peak) const;												//基于给定的峰值点搜索右翼最小值点
	TerrainProfilePoint* _findValley(std::vector<TerrainProfilePoint*>& valleys, RtLbsType tmin, RtLbsType tmax);		//基于最大最小值搜索给定谷底数据集中的最小值
	std::vector<TerrainProfilePoint*> _getPoints(int idMin, int idMax) const;											//寻找最小最大索引对应的坐标点序列	
};

#endif
