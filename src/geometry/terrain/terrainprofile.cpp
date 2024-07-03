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
	
	TerrainPathNode* txNode = new TerrainPathNode(m_txPosition);				/** @brief	tx节点	*/
	outPath->m_nodes.push_back(txNode);											//添加tx节点
	
	for (int i = 0; i < m_ridges.size(); ++i) {									//添加path中的路径节点
		if (m_ridges[i]->m_isValid == false)
			continue;
		TerrainPathNode* newNode = new TerrainPathNode(m_ridges[i]);
		outPath->m_nodes.push_back(newNode);
	}

	TerrainPathNode* rxNode = new TerrainPathNode(m_rxPosition);				/** @brief	rx节点	*/
	outPath->m_nodes.push_back(rxNode);											//添加rx节点

	//更新绕射路径的s参数
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
	RtLbsType hs = m_txPosition.z;										/** @brief	发射点高度	*/
	RtLbsType he = m_rxPosition.z;										/** @brief	接收点高度	*/
    //赋值坐标点
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
		(*it)->m_point2d[1] -= offsetH; //调整剖面高度值
	}
	_calStaticalParameters();								//计算地形剖面统计元素
	if (m_undulation <= 5)									//若地形起伏程度小于5m则认定不存在峰值
		return;
    //计算peaks和valleys
	if (m_points.size() < 3)
		return;
	m_valley.push_back(m_points.front());//在谷底插入起点元素
	for (int i = 1; i < m_points.size() - 1; ++i) {
		TerrainProfilePoint* curPoint = m_points[i];
		TerrainProfilePoint* prevPoint = m_points[i - 1];
		TerrainProfilePoint* nextPoint = m_points[i + 1];
		RtLbsType curZ = curPoint->m_point2d[1];
		RtLbsType prevZ = prevPoint->m_point2d[1];
		RtLbsType nextZ = nextPoint->m_point2d[1];
		if (curZ >= prevZ && curZ >= nextZ && !(curZ == prevZ && curZ == nextZ)) { //寻找全部山峰极值
			if (!_isValidPeak(curPoint))		//无效峰直接跳过
				continue;
			//根据统计特征判定峰值点是否有效
			m_peaks.push_back(curPoint);
		}
		if (curZ < prevZ && curZ < nextZ) { //寻找全部谷底极值
			m_valley.push_back(curPoint);
		}
	}
	m_valley.push_back(m_points.back());//在谷底插入终点元素
}

void TerrainProfile::_calStaticalParameters()
{
	m_maxH = m_points[0]->m_point3d->z;					/** @brief	地形最大值	*/
	m_minH = m_points[0]->m_point3d->z;					/** @brief	地形最小值	*/
	RtLbsType sumH = 0.0;										/** @brief	高度总和	*/
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
	m_meanH = sumH / pNum;										//计算均值
	m_undulation = m_maxH - m_minH;									//计算地形起伏程度
	RtLbsType variance = 0.0;									//方差
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
	RtLbsType distance = p->m_point3d->z - m_minH;				/** @brief	peak 与均值的高度差	*/
	if (distance < m_stdH)										//若高度差值小于标准偏离值，则认为是无效峰
		return false;
	return true;
}

bool TerrainProfile::_getIntersect(TerrainProfileSegment& segment)
{
	std::vector<TerrainProfilePoint*> points = _getPoints(segment.m_ps->pId, segment.m_pe->pId);//循环线段上的每一个坐标点，进行遍历，求解是否相交
	for (auto it = points.begin(); it != points.end(); ++it) {
		TerrainProfilePoint* curPoint = *it;
		if (curPoint->m_point2d[1] >= segment.GetHeight(curPoint)) {//若当前地形高度大于地形剖面线高度，则表明与地形剖面线相交
			return true;
		}
	}
	return false;
}

void TerrainProfile::_findRidges()
{
	RtLbsType hs = m_txPosition.z;													/** @brief	发射点高度	*/
	RtLbsType he = m_rxPosition.z;													/** @brief	接收点高度	*/
	//修正方案-1 
	//针对每个peak 搜索距离最近的两个valley;
	for (int i = 0; i < m_peaks.size(); ++i) {
		TerrainProfilePoint* leftValley = _findLeftValley(m_peaks[i]);				/** @brief	寻找左谷底	*/
		TerrainProfilePoint* rightValley = _findRightValley(m_peaks[i]);			/** @brief	寻找右谷底	*/
		//若左谷底和右谷底分别和峰峦顶的距离低于限值，则放弃该条峰
		RtLbsType leftLen = (m_peaks[i]->m_point2d - leftValley->m_point2d).Length();	/** @brief	峰峦和左底谷间距离	*/
		RtLbsType rightLen = (m_peaks[i]->m_point2d - rightValley->m_point2d).Length();	/** @brief	峰峦和右低谷间距离	*/
		std::vector<TerrainProfilePoint*> points = _getPoints(leftValley->pId, rightValley->pId);
		TerrainRidge* ridge = new TerrainRidge(m_peaks[i], leftValley, rightValley, m_points.front(), m_points.back(), points);
		m_ridges.push_back(ridge);
	}

	if (m_ridges.size() == 0)			//若峰峦数为0则直接返回
		return;
	//对ridges中的数据进行有效性筛除-（设定第一个值为有效值，后续节点按照依次迭代顺序判定间距是否大于5m若大于5m则保留，小于5m则进行放弃）
	TerrainRidge* curRidge = nullptr;						/** @brief	当前的峰峦	*/
	TerrainRidge* preValidRidge = m_ridges.front();			/** @brief	前一个有效峰峦	*/
	//执行峰值筛除算法-1
	for (int i = 1; i < m_ridges.size(); ++i) {
		curRidge = m_ridges[i];
		RtLbsType xyLength = abs(curRidge->m_peak->m_point2d[0] - preValidRidge->m_peak->m_point2d[0]);		//两个峰峦间间距（水平）
		RtLbsType hDiff = curRidge->m_peak->m_point2d[1] - preValidRidge->m_peak->m_point2d[1];				//两个峰峦间高度差
		if (xyLength < 40.0) {							//间距小于30m可以进行替换或失能操作，此项值为山峦和山峦间的最小间距
			if (hDiff >= 0 || abs(hDiff) <= EPSILON) {
				preValidRidge->m_isValid = false;		//若当前峦高于前峦，前一有效峦失效
				preValidRidge = curRidge;				//当前峦有效,进行有效峦替换
			}
			else {
				curRidge->m_isValid = false;			//若当前峦低于前峦，当前峦失效	
			}
			continue;
		}
		preValidRidge = curRidge;						//若距离小于判定阈值，则当前峦为有效峦，进行有效峦替换
	}

	//统计有效峰峦数
	for (auto it = m_ridges.begin(); it != m_ridges.end(); ++it) {
		const TerrainRidge* ridge = *it;
		if (ridge->m_isValid == true)
			m_validRidgeNum++;
	}
}

TerrainProfilePoint* TerrainProfile::_findLeftValley(TerrainProfilePoint* peak) const
{
	int minIdDistance = INT_MAX;
	TerrainProfilePoint* minLeftValley = nullptr;				/** @brief	需要返回的左边最小谷底	*/
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
	TerrainProfilePoint* minRightValley = nullptr;				/** @brief	需要返回的右边最小谷底	*/
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
	RtLbsType minH = FLT_MAX; //初始化最小谷底高度,默认最大值
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
