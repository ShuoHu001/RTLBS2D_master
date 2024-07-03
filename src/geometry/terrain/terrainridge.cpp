#include "terrainridge.h"

TerrainRidge::TerrainRidge()
	: m_isValid(true)
	, m_peak(nullptr)
	, m_leftValley(nullptr)
	, m_rightValley(nullptr)
	, m_curvRadius(0.0)
	, m_actualHeight(0.0)
	, m_relativeHeight(0.0)
	, m_mat(nullptr)
{
}

TerrainRidge::TerrainRidge(TerrainProfilePoint* peak, TerrainProfilePoint* leftValley, TerrainProfilePoint* rightValley, TerrainProfilePoint* sp, TerrainProfilePoint* ep, std::vector<TerrainProfilePoint*> points)
	: m_isValid(true)
	, m_peak(peak)
	, m_leftValley(leftValley)
	, m_rightValley(rightValley)
	, m_points(points)
	, m_mat(peak->m_mat)
{
	if (points.size() < 3)
		LOG_ERROR << "num must be large than 3." << CRASH;
	m_actualHeight = peak->m_point2d[1];				//真实的峰顶高度
	//计算相对峰峦高度-封顶在收发连线上的高度
	m_relativeHeight = _calPeakRelativeHeightInTRX(sp, ep);
	//计算曲率半径（中值公式法）
	m_curvRadius = _calCurvRadius();
}

TerrainRidge::TerrainRidge(const TerrainRidge& ridge)
	: m_isValid(ridge.m_isValid)
	, m_peak(ridge.m_peak)
	, m_points(ridge.m_points)
	, m_leftValley(ridge.m_leftValley)
	, m_rightValley(ridge.m_rightValley)
	, m_curvRadius(ridge.m_curvRadius)
	, m_actualHeight(ridge.m_actualHeight)
	, m_relativeHeight(ridge.m_relativeHeight)
	, m_mat(ridge.m_mat)
{
}

TerrainRidge::~TerrainRidge()
{
}

TerrainRidge& TerrainRidge::operator=(const TerrainRidge& ridge)
{
	m_isValid = ridge.m_isValid;
	m_points = ridge.m_points;
	m_peak = ridge.m_peak;
	m_leftValley = ridge.m_leftValley;
	m_rightValley = ridge.m_rightValley;
	m_curvRadius = ridge.m_curvRadius;
	m_actualHeight = ridge.m_actualHeight;
	m_mat = ridge.m_mat;
	return *this;
}

bool TerrainRidge::operator==(const TerrainRidge& other) const
{
	return m_peak == other.m_peak;
}

bool TerrainRidge::operator!=(const TerrainRidge& other) const
{
	return !(*this == other);
}

void TerrainRidge::Init(TerrainProfilePoint* peak, TerrainProfilePoint* leftValley, TerrainProfilePoint* rightValley, TerrainProfilePoint* sp, TerrainProfilePoint* ep, std::vector<TerrainProfilePoint*> points)
{
	m_peak = peak;
	m_leftValley = leftValley;
	m_rightValley = rightValley;
	m_points = points;
	m_actualHeight = peak->m_point2d[1];
	//计算相对峰峦高度-封顶在收发连线上的高度
	m_relativeHeight = _calPeakRelativeHeightInTRX(sp, ep);
	m_curvRadius = _calCurvRadius();	//计算曲率半径（中值公式法）
	m_mat = peak->m_mat;

	//判定ridge是否有效 判定标准为谷底和峰之间间隔超过5m，峰高度超过5m 需要进行确定和研究的点
	if (abs(rightValley->m_point2d[1] - leftValley->m_point2d[1]) < 5.0 || abs(m_relativeHeight) < 5.0) {
		m_isValid = false;
	}
}

RtLbsType TerrainRidge::GetNValue() const
{
	//计算外角值
	Vector3D ps = *m_peak->m_point3d - *m_leftValley->m_point3d;									/** @brief	峰指向左谷的向量	*/
	Vector3D pe = *m_peak->m_point3d - *m_rightValley->m_point3d;									/** @brief	峰指向右谷的向量	*/
	RtLbsType costheta = ps.Normalize() * pe.Normalize();											/** @brief	峰峦角余弦值	*/
	RtLbsType theta = acos(costheta);																/** @brief	峰峦角	*/
	return 2 - theta / PI;																			//返回N值
}

void TerrainRidge::CalDiffractionParameters(const Vector3D& inDir, const Vector3D& diffDir, RtLbsType& incidentTheta, RtLbsType& diffractionTheta) const
{
	//0面为靠近左谷的面
	Vector3D zeroFaceDir = (*m_peak->m_point3d - *m_leftValley->m_point3d).Normalize();							//0面向量
	RtLbsType incidentCosTheta = zeroFaceDir * inDir;															//入射角余弦
	incidentTheta = acos(incidentCosTheta);																		//求解入射角
	//判定绕射角为锐角还是钝角
	RtLbsType diffCosTheta = zeroFaceDir * diffDir;																//绕射角余弦
	RtLbsType dTheta = acos(diffractionTheta);																	//绕射角
	RtLbsType crossFlag = Cross(zeroFaceDir, inDir) * Cross(zeroFaceDir, diffDir);
	if (crossFlag < 0) {			//与入射角在0面的不同侧，绕射角为锐角
		diffractionTheta = dTheta;
	}
	else {							//与入射角在0面同侧，绕射角为钝角
		diffractionTheta = dTheta + PI;
	}
}

void TerrainRidge::WriteToFile(std::ofstream& outFile) const
{
	//写入峰峦坐标
	outFile << m_peak->m_point3d->x << "\t" << m_peak->m_point3d->y << "\t" << m_peak->m_point3d->z<<"\t";
	//写入谷底坐标
	outFile << m_leftValley->m_point3d->x << "\t" << m_leftValley->m_point3d->y << "\t" << m_leftValley->m_point3d->z << "\t";
	outFile << m_rightValley->m_point3d->x << "\t" << m_rightValley->m_point3d->y << "\t" << m_rightValley->m_point3d->z << "\t";
	//写入相对高度（净空高度）
	outFile << m_relativeHeight << "\t";
	//写入是否有效
	outFile << m_isValid << std::endl;
}

RtLbsType TerrainRidge::_calCurvRadius()
{
	//计算峰值临近点的法向量平均值
	TerrainProfilePoint* curProfilePoint = m_peak;
	TerrainProfilePoint* prevProfilePoint = _findPoint(m_peak->pId - 1);
	TerrainProfilePoint* nextProfilePoint = _findPoint(m_peak->pId + 1);
	//此处可增加有效性判定，目前先不加
	Point2D& curP = m_peak->m_point2d;
	Point2D& prevP = prevProfilePoint->m_point2d;
	Point2D& nextP = nextProfilePoint->m_point2d;

	Vector2D ab = curP - prevP;
	Vector2D bc = nextP - curP;

	RtLbsType l_ab = ab.Length();
	RtLbsType l_bc = ab.Length();

	//计算单位法向量
	Vector2D n_ab, n_bc;
	n_ab.x = -ab.y / l_ab;
	n_ab.y = ab.x / l_ab;
	n_bc.x = -bc.y / l_bc;
	n_bc.y = bc.x / l_bc;

	RtLbsType angle = acos(n_ab * n_bc);
	RtLbsType curvRadius = 1.0 / (2.0 * sin(angle / 2.0) * l_bc);
	return curvRadius;
}

TerrainProfilePoint* TerrainRidge::_findPoint(int id) const
{
	int idmin = m_points.front()->pId;
	int idmax = m_points.back()->pId;
	if (id < idmin || id > idmax)
		return nullptr;
	int offset = id - idmin; //计算id在数组中的偏移值
	return m_points[offset];
}

RtLbsType TerrainRidge::_calPeakRelativeHeightInTRX(TerrainProfilePoint* sp, TerrainProfilePoint* ep) const
{
	RtLbsType& hp = m_peak->m_point2d[1];				/** @brief	p 点高度	*/
	RtLbsType& hs = sp->m_point2d[1];					/** @brief	起始点高度	*/
	RtLbsType& he = ep->m_point2d[1];					/** @brief	终止点高度	*/
	RtLbsType h = he - hs;								/** @brief	以起始点高度为参考，计算距离终止点的相对高度	*/
	if (abs(h) < EPSILON) {					//起点和终点所构成的线段斜率为0
		return hp - hs;		//返回p点相对高度
	}
	RtLbsType s1 = m_peak->m_point2d[0] - sp->m_point2d[0];		/** @brief	p 点距离起始点的水平距离	*/
	RtLbsType s = ep->m_point2d[0] - sp->m_point2d[0];			/** @brief	起始点距离终止点的水平距离	*/
	RtLbsType hrp = hp - s1 / s * he - (s1 - s) / s * hs;		/** @brief	按照比例关系计算p点垂线距离	*/
	return hrp;
}
