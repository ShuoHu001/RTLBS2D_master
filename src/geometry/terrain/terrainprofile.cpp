#include "terrainprofile.h"

TerrainProfile::TerrainProfile()
	: m_validRidgeNum(0)
	, m_minH(0.0)
	, m_maxH(0.0)
	, m_meanH(0.0)
	, m_stdH(0.0)
	, m_undulation(0.0)
{
}

TerrainProfile::~TerrainProfile()
{
}

void TerrainProfile::InitParameters(const std::vector<Point3D>& points, const std::vector<Material*>& mats, const Point3D& txPosition, const Point3D& rxPosition, RtLbsType averRidgeGap)
{
	m_txPosition = txPosition;
	m_rxPosition = rxPosition;
	_init(points, mats);
	_findRidges();
}

void TerrainProfile::GetDiffractPathOverRidges(TerrainDiffractionPath*& outPath) const
{
	if (m_validRidgeNum == 0)
		return;
	outPath = new TerrainDiffractionPath();
	
	TerrainPathNode* txNode = new TerrainPathNode(m_txPosition);				/** @brief	tx�ڵ�	*/
	outPath->m_nodes.push_back(txNode);											//���tx�ڵ�
	
	for (int i = 0; i < m_ridges.size(); ++i) {									//���path�е�·���ڵ�
		if (m_ridges[i]->m_isValid == false)
			continue;
		TerrainPathNode* newNode = new TerrainPathNode(m_ridges[i]);
		outPath->m_nodes.push_back(newNode);
	}

	TerrainPathNode* rxNode = new TerrainPathNode(m_rxPosition);				/** @brief	rx�ڵ�	*/
	outPath->m_nodes.push_back(rxNode);											//���rx�ڵ�

	//��������·����s����
	outPath->RectifySParameters();
}

void TerrainProfile::WritePeaksToFile(std::string filename) const
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_ERROR << filename << " :can't open file." << ENDL;
		return;
	}
	for (auto it = m_peaks.begin(); it != m_peaks.end(); ++it) {
		const Point3D* point = (*it)->m_point3d;
		outFile << point->x << "," << point->y << "," << point->z << std::endl;
	}
	outFile.close();
}

void TerrainProfile::WriteValleysToFile(std::string filename) const
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_ERROR << filename << " :can't open file." << ENDL;
		return;
	}
	for (auto it = m_valley.begin(); it != m_valley.end(); ++it) {
		const Point3D* point = (*it)->m_point3d;
		outFile << point->x << "," << point->y << "," << point->z << std::endl;
	}
	outFile.close();
}

void TerrainProfile::WriteRidgesToFile(std::string filename) const
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_ERROR << filename << " :can't open file." << ENDL;
		return;
	}
	for (auto it = m_ridges.begin(); it != m_ridges.end(); ++it) {
		TerrainRidge* ridge = *it;
		ridge->WriteToFile(outFile);
	}
	outFile.close();
}

void TerrainProfile::WriteProfileToFile(std::string filename) const
{
	std::ofstream outFile(filename);
	if (!outFile.is_open()) {
		LOG_ERROR << filename << " :can't open file." << ENDL;
		return;
	}
	for (auto it = m_points.begin(); it != m_points.end(); ++it) {
		TerrainProfilePoint* point = *it;
		point->WriteToFile(outFile);
	}
	outFile.close();
}

void TerrainProfile::_init(const std::vector<Point3D>& points, const std::vector<Material*>& mats)
{
	RtLbsType hs = m_txPosition.z;										/** @brief	�����߶�	*/
	RtLbsType he = m_rxPosition.z;										/** @brief	���յ�߶�	*/
    //��ֵ�����
	m_points.resize(points.size());
	const Point3D& startPoint = points.front();
	const Point3D& endPoint = points.back();
	for (int i = 0; i < points.size(); ++i) {
		TerrainProfilePoint* point = new TerrainProfilePoint(i, &points[i], &startPoint, mats[i]);
		m_points[i] = point;
	}

	TerrainProfilePoint* pointStart = new TerrainProfilePoint(*m_points.front());
	pointStart->m_point2d[1] = hs;
	TerrainProfilePoint* pointEnd = new TerrainProfilePoint(*m_points.back());
	pointEnd->m_point2d[1] = he;
	TerrainProfileSegment segment(pointStart, pointEnd);
	for (auto it = m_points.begin(); it != m_points.end(); ++it) {
		RtLbsType offsetH = segment.GetHeight(*it);
		(*it)->m_point2d[1] -= offsetH; //��������߶�ֵ
	}
	_calStaticalParameters();								//�����������ͳ��Ԫ��
	if (m_undulation <= 5)									//����������̶�С��5m���϶������ڷ�ֵ
		return;
    //����peaks��valleys
	if (m_points.size() < 3)
		return;
	m_valley.push_back(m_points.front());//�ڹȵײ������Ԫ��
	for (int i = 1; i < m_points.size() - 1; ++i) {
		TerrainProfilePoint* curPoint = m_points[i];
		TerrainProfilePoint* prevPoint = m_points[i - 1];
		TerrainProfilePoint* nextPoint = m_points[i + 1];
		RtLbsType curZ = curPoint->m_point2d[1];
		RtLbsType prevZ = prevPoint->m_point2d[1];
		RtLbsType nextZ = nextPoint->m_point2d[1];
		if (curZ >= prevZ && curZ >= nextZ && !(curZ == prevZ && curZ == nextZ)) { //Ѱ��ȫ��ɽ�弫ֵ
			if (!_isValidPeak(curPoint))		//��Ч��ֱ������
				continue;
			//����ͳ�������ж���ֵ���Ƿ���Ч
			m_peaks.push_back(curPoint);
		}
		if (curZ < prevZ && curZ < nextZ) { //Ѱ��ȫ���ȵ׼�ֵ
			m_valley.push_back(curPoint);
		}
	}
	m_valley.push_back(m_points.back());//�ڹȵײ����յ�Ԫ��
}

void TerrainProfile::_calStaticalParameters()
{
	m_maxH = m_points[0]->m_point3d->z;					/** @brief	�������ֵ	*/
	m_minH = m_points[0]->m_point3d->z;					/** @brief	������Сֵ	*/
	RtLbsType sumH = 0.0;										/** @brief	�߶��ܺ�	*/
	for (auto it = m_points.begin(); it != m_points.end(); ++it) {
		const TerrainProfilePoint* point = *it;
		const RtLbsType& curH = point->m_point3d->z;
		if (curH > m_maxH)
			m_maxH = curH;
		if (curH < m_minH)
			m_minH = curH;
		sumH += curH;
	}
	int pNum = static_cast<int>(m_points.size());
	m_meanH = sumH / pNum;										//�����ֵ
	m_undulation = m_maxH - m_minH;									//�����������̶�
	RtLbsType variance = 0.0;									//����
	for (auto it = m_points.begin(); it != m_points.end(); ++it) {
		const TerrainProfilePoint* point = *it;
		const RtLbsType& curH = point->m_point3d->z;
		variance += (curH - m_meanH) * (curH - m_meanH);
	}
	variance /= m_points.size();
	m_stdH = sqrt(variance);
}

bool TerrainProfile::_isValidPeak(const TerrainProfilePoint* p) const
{
	RtLbsType distance = p->m_point3d->z - m_minH;				/** @brief	peak ���ֵ�ĸ߶Ȳ�	*/
	if (distance < m_stdH)										//���߶Ȳ�ֵС�ڱ�׼ƫ��ֵ������Ϊ����Ч��
		return false;
	return true;
}

bool TerrainProfile::_getIntersect(TerrainProfileSegment& segment)
{
	std::vector<TerrainProfilePoint*> points = _getPoints(segment.m_ps->pId, segment.m_pe->pId);//ѭ���߶��ϵ�ÿһ������㣬���б���������Ƿ��ཻ
	for (auto it = points.begin(); it != points.end(); ++it) {
		TerrainProfilePoint* curPoint = *it;
		if (curPoint->m_point2d[1] >= segment.GetHeight(curPoint)) {//����ǰ���θ߶ȴ��ڵ��������߸߶ȣ������������������ཻ
			return true;
		}
	}
	return false;
}

void TerrainProfile::_findRidges()
{
	RtLbsType hs = m_txPosition.z;													/** @brief	�����߶�	*/
	RtLbsType he = m_rxPosition.z;													/** @brief	���յ�߶�	*/
	//��������-1 
	//���ÿ��peak �����������������valley;
	for (int i = 0; i < m_peaks.size(); ++i) {
		TerrainProfilePoint* leftValley = _findLeftValley(m_peaks[i]);				/** @brief	Ѱ����ȵ�	*/
		TerrainProfilePoint* rightValley = _findRightValley(m_peaks[i]);			/** @brief	Ѱ���ҹȵ�	*/
		//����ȵ׺��ҹȵ׷ֱ�ͷ��Ͷ��ľ��������ֵ�������������
		RtLbsType leftLen = (m_peaks[i]->m_point2d - leftValley->m_point2d).Length();	/** @brief	���ͺ���׹ȼ����	*/
		RtLbsType rightLen = (m_peaks[i]->m_point2d - rightValley->m_point2d).Length();	/** @brief	���ͺ��ҵ͹ȼ����	*/
		std::vector<TerrainProfilePoint*> points = _getPoints(leftValley->pId, rightValley->pId);
		TerrainRidge* ridge = new TerrainRidge(m_peaks[i], leftValley, rightValley, m_points.front(), m_points.back(), points);
		m_ridges.push_back(ridge);
	}

	if (m_ridges.size() == 0)			//��������Ϊ0��ֱ�ӷ���
		return;
	//��ridges�е����ݽ�����Ч��ɸ��-���趨��һ��ֵΪ��Чֵ�������ڵ㰴�����ε���˳���ж�����Ƿ����5m������5m������С��5m����з�����
	TerrainRidge* curRidge = nullptr;						/** @brief	��ǰ�ķ���	*/
	TerrainRidge* preValidRidge = m_ridges.front();			/** @brief	ǰһ����Ч����	*/
	//ִ�з�ֵɸ���㷨-1
	for (int i = 1; i < m_ridges.size(); ++i) {
		curRidge = m_ridges[i];
		RtLbsType xyLength = abs(curRidge->m_peak->m_point2d[0] - preValidRidge->m_peak->m_point2d[0]);		//�������ͼ��ࣨˮƽ��
		RtLbsType hDiff = curRidge->m_peak->m_point2d[1] - preValidRidge->m_peak->m_point2d[1];				//�������ͼ�߶Ȳ�
		if (xyLength < 40.0) {							//���С��30m���Խ����滻��ʧ�ܲ���������ֵΪɽ�ͺ�ɽ�ͼ����С���
			if (hDiff >= 0 || abs(hDiff) <= EPSILON) {
				preValidRidge->m_isValid = false;		//����ǰ�͸���ǰ�ͣ�ǰһ��Ч��ʧЧ
				preValidRidge = curRidge;				//��ǰ����Ч,������Ч���滻
			}
			else {
				curRidge->m_isValid = false;			//����ǰ�͵���ǰ�ͣ���ǰ��ʧЧ	
			}
			continue;
		}
		preValidRidge = curRidge;						//������С���ж���ֵ����ǰ��Ϊ��Ч�ͣ�������Ч���滻
	}

	//ͳ����Ч������
	for (auto it = m_ridges.begin(); it != m_ridges.end(); ++it) {
		const TerrainRidge* ridge = *it;
		if (ridge->m_isValid == true)
			m_validRidgeNum++;
	}
}

TerrainProfilePoint* TerrainProfile::_findLeftValley(TerrainProfilePoint* peak) const
{
	int minIdDistance = INT_MAX;
	TerrainProfilePoint* minLeftValley = nullptr;				/** @brief	��Ҫ���ص������С�ȵ�	*/
	for (int i = 0; i < m_valley.size(); ++i) {
		if (m_valley[i]->pId >= peak->pId) {
			continue;
		}
		if ((peak->pId - m_valley[i]->pId) < minIdDistance) {
			minIdDistance = peak->pId - m_valley[i]->pId;
			minLeftValley = m_valley[i];
		}
	}
	return minLeftValley;
}

TerrainProfilePoint* TerrainProfile::_findRightValley(TerrainProfilePoint* peak) const
{
	int minIdDistance = INT_MAX;
	TerrainProfilePoint* minRightValley = nullptr;				/** @brief	��Ҫ���ص��ұ���С�ȵ�	*/
	for (int i = 0; i < m_valley.size(); ++i) {
		if (m_valley[i]->pId <= peak->pId) {
			continue;
		}
		if ((m_valley[i]->pId - peak->pId) < minIdDistance) {
			minIdDistance = m_valley[i]->pId - peak->pId;
			minRightValley = m_valley[i];
		}
	}
	return minRightValley;
}

TerrainProfilePoint* TerrainProfile::_findValley(std::vector<TerrainProfilePoint*>& valleys, RtLbsType tmin, RtLbsType tmax)
{
	TerrainProfilePoint* minValley = nullptr;
	RtLbsType minH = FLT_MAX; //��ʼ����С�ȵ׸߶�,Ĭ�����ֵ
	for (auto it = valleys.begin(); it != valleys.end(); ++it) {
		TerrainProfilePoint* curValley = *it;
		if (curValley->m_point2d[0] >= tmin && curValley->m_point2d[0] <= tmax) {
			if (curValley->m_point2d[1] < minH) {
				minValley = curValley;
				minH = curValley->m_point2d[1];
			}
		}
	}
	return minValley;
}

std::vector<TerrainProfilePoint*> TerrainProfile::_getPoints(int idMin, int idMax) const
{
	if (idMin >= idMax) {
		LOG_ERROR << "TerrainProfile: idMin>idMax , wrong." << CRASH;
	}
	std::vector<TerrainProfilePoint*> reVal;
	for (auto it = m_points.begin(); it != m_points.end(); ++it) {
		TerrainProfilePoint* p = *it;
		if (p->pId >= idMin && p->pId <= idMax) {
			reVal.push_back(p);
		}
	}
	return reVal;
}
