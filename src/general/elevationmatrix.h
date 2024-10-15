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
	bool Init(const std::string& filename, int mode);							//���ļ��г�ʼ��������,mode=0��ʾ������ʽ��ȡ��mode=1��ʾ���е���ʽ��ȡ
	RtLbsType GetValue(const Point2D& p) const;									//��ȡλ�õ�p���ڵ�ֵ
	RtLbsType GetValue(const Point3D& p) const;									//��ȡλ�õ�p���ڵ�ֵ

};

#endif