#include "wedge2d.h"
#include "segment2d.h"

Wedge2D::Wedge2D()
	: m_globalId(-1)
	, m_privateId(-1)
	, m_face1(nullptr)
	, m_face2(nullptr)
	, m_theta(0.0)
{
}

Wedge2D::Wedge2D(int privateId, Segment2D* face1, Segment2D* face2, Point2D p)
	: m_globalId(-1)
	, m_privateId(privateId)
	, m_face1(face1)
	, m_face2(face2)
	, m_point(p)
{
	m_dir1 = face1->m_dir;
	m_dir2 = face2->m_dir;
	//�����ڽǻ���
	if (m_face1->m_pe == p) {//��Ҫ����vector����
		m_dir1 = -m_face1->m_dir;
	}
	if (m_face2->m_pe == p) {//��Ҫ����vector����
		m_dir2 = -m_face2->m_dir;
	}
	double theta = acos(m_dir1 * m_dir2);
	m_theta = TWO_PI - theta;//�����ǣ�����180��
	Vector2D nsum = (m_face1->m_normal + m_face2->m_normal) / 2.0;
	if (Cross(nsum, m_face1->m_normal) * Cross(nsum, m_dir1) < 0.0) {//�������ڽǳ���180�ȣ���Ш�ν����Ϊԭʼ�����
		m_theta = theta;
	}
}

Wedge2D::Wedge2D(int privateId, Segment2D* face1, Segment2D* face2, Point2D p, double theta, Vector2D vector1, Vector2D vector2)
	: m_globalId(-1)
	, m_privateId(privateId)
	, m_face1(face1)
	, m_face2(face2)
	, m_point(p)
	, m_theta(theta)
	, m_dir1(vector1)
	, m_dir2(vector2)
{}

bool Wedge2D::operator==(const Wedge2D& other) const
{
	if (m_globalId != other.m_globalId)
		return false;
	return true;
}

bool Wedge2D::operator!=(const Wedge2D& other) const
{
	return !(*this == other);
}

Wedge2D& Wedge2D::operator=(const Wedge2D& wedge)
{
	if (this != &wedge) {
		m_globalId = wedge.m_globalId;
		m_face1 = wedge.m_face1;
		m_face2 = wedge.m_face2;
		m_point = wedge.m_point;
		m_theta = wedge.m_theta;
		m_dir1 = wedge.m_dir1;
		m_dir2 = wedge.m_dir2;
	}
	return *this;
}

bool Wedge2D::IsContainsPoint(const Point2D& point)
{
	if (m_face1->m_ps == point || m_face1->m_pe == point || m_face2->m_ps == point || m_face2->m_pe == point) {
		return true;
	}
	return false;
}

Wedge2DGPU Wedge2D::Convert2GPU()
{
	Wedge2DGPU wedge;
	wedge.m_wedge_id = m_globalId;
	wedge.m_face1 = m_face1->Convert2GPU();
	wedge.m_face2 = m_face2->Convert2GPU();
	wedge.m_edge = m_point;
	wedge.m_fExternalAngle = m_theta;
	wedge.m_vector1 = m_dir1;
	wedge.m_vector2 = m_dir2;
	return wedge;
}

RtLbsType Wedge2D::GetNValue() const
{
	return m_theta / PI;
}

void Wedge2D::CalDiffractionParameters(const Vector3D& inDir, const Vector3D& diffDir, const Point3D& interPoint, RtLbsType& incidentTheta, RtLbsType& diffractionTheta) const
{
	//1-ȷ��0��
	Vector2D inDir2D = Vector2D(inDir.x, inDir.y).Normalize();					/** @brief	��ά���䷽������	*/
	Vector2D diffDir2D = Vector2D(diffDir.x, diffDir.y).Normalize();			/** @brief	��ά���䷽������	*/

	Vector2D midDir = -1 * (m_dir1 + m_dir2).Normalize();						/** @brief	�߲෽�������ϲ��ķ�����	*/
	Vector2D leftSideDir;														/** @brief	��߲�����	*/
	Vector2D rightSideDir;														/** @brief	�ұ߲�����	*/

	if (Cross(midDir, m_dir1) > 0) {											//ȷ������������Ҳ�����
		leftSideDir = m_dir1;
		rightSideDir = m_dir2;
	}
	else {
		leftSideDir = m_dir2;
		rightSideDir = m_dir1;
	}

	Vector2D zeroFace;															//0�淽��
	Vector2D nFace;																//n�淽��
	if (Cross(midDir, inDir2D) >= 0) {											//�����
		zeroFace = leftSideDir;
		nFace = rightSideDir;
	}
	else {																		//���Ҳ�
		zeroFace = rightSideDir;
		nFace = leftSideDir;
	}

	RtLbsType cosInTheta = zeroFace * inDir2D;
	if (cosInTheta > 1.0) cosInTheta = 1.0;
	if (cosInTheta < -1.0) cosInTheta = -1.0;
	incidentTheta = acos(cosInTheta);

	if (Cross(midDir, diffDir2D) >= 0) {										//���䷽�������
		RtLbsType cosDiffTheta = diffDir2D * leftSideDir;
		if (cosDiffTheta > 1.0) cosDiffTheta = 1.0;
		if (cosDiffTheta < -1.0) cosDiffTheta = -1.0;
		diffractionTheta = acos(cosDiffTheta);
	}
	else {																		//���䷽�����Ҳ�
		RtLbsType cosDiffTheta = diffDir2D * rightSideDir;
		if (cosDiffTheta > 1.0) cosDiffTheta = 1.0;
		if (cosDiffTheta < -1.0) cosDiffTheta = -1.0;
		diffractionTheta = m_theta - acos(cosDiffTheta);
	}

}
