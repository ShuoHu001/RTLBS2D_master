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
	m_actualHeight = peak->m_point2d[1];				//��ʵ�ķ嶥�߶�
	//������Է��͸߶�-�ⶥ���շ������ϵĸ߶�
	m_relativeHeight = _calPeakRelativeHeightInTRX(sp, ep);
	//�������ʰ뾶����ֵ��ʽ����
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
	//������Է��͸߶�-�ⶥ���շ������ϵĸ߶�
	m_relativeHeight = _calPeakRelativeHeightInTRX(sp, ep);
	m_curvRadius = _calCurvRadius();	//�������ʰ뾶����ֵ��ʽ����
	m_mat = peak->m_mat;

	//�ж�ridge�Ƿ���Ч �ж���׼Ϊ�ȵ׺ͷ�֮��������5m����߶ȳ���5m ��Ҫ����ȷ�����о��ĵ�
	if (abs(rightValley->m_point2d[1] - leftValley->m_point2d[1]) < 5.0 || abs(m_relativeHeight) < 5.0) {
		m_isValid = false;
	}
}

RtLbsType TerrainRidge::GetNValue() const
{
	//�������ֵ
	Vector3D ps = *m_peak->m_point3d - *m_leftValley->m_point3d;									/** @brief	��ָ����ȵ�����	*/
	Vector3D pe = *m_peak->m_point3d - *m_rightValley->m_point3d;									/** @brief	��ָ���ҹȵ�����	*/
	RtLbsType costheta = ps.Normalize() * pe.Normalize();											/** @brief	���ͽ�����ֵ	*/
	RtLbsType theta = acos(costheta);																/** @brief	���ͽ�	*/
	return 2 - theta / PI;																			//����Nֵ
}

void TerrainRidge::CalDiffractionParameters(const Vector3D& inDir, const Vector3D& diffDir, RtLbsType& incidentTheta, RtLbsType& diffractionTheta) const
{
	//0��Ϊ������ȵ���
	Vector3D zeroFaceDir = (*m_peak->m_point3d - *m_leftValley->m_point3d).Normalize();							//0������
	RtLbsType incidentCosTheta = zeroFaceDir * inDir;															//���������
	incidentTheta = acos(incidentCosTheta);																		//��������
	//�ж������Ϊ��ǻ��Ƕ۽�
	RtLbsType diffCosTheta = zeroFaceDir * diffDir;																//���������
	RtLbsType dTheta = acos(diffractionTheta);																	//�����
	RtLbsType crossFlag = Cross(zeroFaceDir, inDir) * Cross(zeroFaceDir, diffDir);
	if (crossFlag < 0) {			//���������0��Ĳ�ͬ�࣬�����Ϊ���
		diffractionTheta = dTheta;
	}
	else {							//���������0��ͬ�࣬�����Ϊ�۽�
		diffractionTheta = dTheta + PI;
	}
}

void TerrainRidge::WriteToFile(std::ofstream& outFile) const
{
	//д���������
	outFile << m_peak->m_point3d->x << "\t" << m_peak->m_point3d->y << "\t" << m_peak->m_point3d->z<<"\t";
	//д��ȵ�����
	outFile << m_leftValley->m_point3d->x << "\t" << m_leftValley->m_point3d->y << "\t" << m_leftValley->m_point3d->z << "\t";
	outFile << m_rightValley->m_point3d->x << "\t" << m_rightValley->m_point3d->y << "\t" << m_rightValley->m_point3d->z << "\t";
	//д����Ը߶ȣ����ո߶ȣ�
	outFile << m_relativeHeight << "\t";
	//д���Ƿ���Ч
	outFile << m_isValid << std::endl;
}

RtLbsType TerrainRidge::_calCurvRadius()
{
	//�����ֵ�ٽ���ķ�����ƽ��ֵ
	TerrainProfilePoint* curProfilePoint = m_peak;
	TerrainProfilePoint* prevProfilePoint = _findPoint(m_peak->pId - 1);
	TerrainProfilePoint* nextProfilePoint = _findPoint(m_peak->pId + 1);
	//�˴���������Ч���ж���Ŀǰ�Ȳ���
	Point2D& curP = m_peak->m_point2d;
	Point2D& prevP = prevProfilePoint->m_point2d;
	Point2D& nextP = nextProfilePoint->m_point2d;

	Vector2D ab = curP - prevP;
	Vector2D bc = nextP - curP;

	RtLbsType l_ab = ab.Length();
	RtLbsType l_bc = ab.Length();

	//���㵥λ������
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
	int offset = id - idmin; //����id�������е�ƫ��ֵ
	return m_points[offset];
}

RtLbsType TerrainRidge::_calPeakRelativeHeightInTRX(TerrainProfilePoint* sp, TerrainProfilePoint* ep) const
{
	RtLbsType& hp = m_peak->m_point2d[1];				/** @brief	p ��߶�	*/
	RtLbsType& hs = sp->m_point2d[1];					/** @brief	��ʼ��߶�	*/
	RtLbsType& he = ep->m_point2d[1];					/** @brief	��ֹ��߶�	*/
	RtLbsType h = he - hs;								/** @brief	����ʼ��߶�Ϊ�ο������������ֹ�����Ը߶�	*/
	if (abs(h) < EPSILON) {					//�����յ������ɵ��߶�б��Ϊ0
		return hp - hs;		//����p����Ը߶�
	}
	RtLbsType s1 = m_peak->m_point2d[0] - sp->m_point2d[0];		/** @brief	p �������ʼ���ˮƽ����	*/
	RtLbsType s = ep->m_point2d[0] - sp->m_point2d[0];			/** @brief	��ʼ�������ֹ���ˮƽ����	*/
	RtLbsType hrp = hp - s1 / s * he - (s1 - s) / s * hs;		/** @brief	���ձ�����ϵ����p�㴹�߾���	*/
	return hrp;
}
