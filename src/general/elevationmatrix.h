#ifndef RTLBS_ERRORMATRIX
#define RTLBS_ERRORMATRIX

#include "rtlbs.h"
#include "utility/define.h"
#include "math/point2d.h"
#include "math/point3d.h"
#include "managers/logmanager.h"

class ElevationMatrix {
private:
	int m_xNum;
	int m_yNum;
	RtLbsType m_xMin;
	RtLbsType m_yMin;
	RtLbsType m_xMax;
	RtLbsType m_yMax;
	RtLbsType m_gap;
	std::vector<RtLbsType> m_data;
	
public:
	ElevationMatrix();
	~ElevationMatrix();
	bool Init(const std::string& filename, int mode);							//从文件中初始化误差矩阵,mode=0表示矩阵形式读取；mode=1表示序列点形式读取
	RtLbsType GetValue(const Point2D& p) const;									//获取位置点p所在的值
	RtLbsType GetValue(const Point3D& p) const;									//获取位置点p所在的值

};

#endif