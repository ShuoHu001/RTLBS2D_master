#include "building2d.h"
#include "geometry/segment2d.h"

Building2D::Building2D()
	: Object2D()
	, m_buildingId(-1)
{
	m_category = BUILDING2D;
}

Building2D::~Building2D()
{
}

bool LoadBuildingsFromFile(const std::string& geometryFile, const std::string& attributeFile, std::vector<Building2D*>& outBuildings, RtLbsType positionError)
{
	if (geometryFile.empty() || attributeFile.empty()) {
		LOG_ERROR << "Building2D: empty filename." << ENDL;
		return false;
	}
	std::ifstream gstream(geometryFile);					/** @brief	�����ļ���ȡ��	*/
	if (!gstream.is_open()) {
		LOG_ERROR << "Building2D: failed to open " << geometryFile << " ." << ENDL;
		return false;
	}

	std::ifstream astream(attributeFile);					/** @brief	�����ļ���ȡ��	*/
	if (!astream.is_open()) {
		LOG_ERROR << "Building2D: failed to open " << attributeFile << " ." << ENDL;
		return false;
	}

	//��ȡ�����ļ�����
	std::stringstream gss;
	gss << gstream.rdbuf();
	gstream.close();
	std::vector<std::string> glines;
	std::string gline;
	while (std::getline(gss, gline)) {
		if(gline.empty())
			continue;
		glines.push_back(gline);
	}
	int buildingId = 0;
	
	for (int i = 0; i < glines.size(); ++i) {
		std::vector<std::string> cols;
		
		boost::split(cols, glines[i], boost::is_any_of(" \t,"));
		if (cols.size() == 3) {			//building ʶ���ļ�,ǰ��Ϊ�㲻Ϊ��
			//��ȡ�㼯����
			int pNum = boost::lexical_cast<int>(cols[2]);
			std::vector<Point2D> points(pNum);								/** @brief	��ȡ�ĵ�����	*/
			for (int j = 0; j < pNum; ++j) {
				std::vector<std::string> cols1;
				boost::split(cols1, glines[++i], boost::is_any_of(" \t,"));
				RtLbsType x = boost::lexical_cast<RtLbsType>(cols1[0]);
				RtLbsType y = boost::lexical_cast<RtLbsType>(cols1[1]);
				if (positionError != 0) {						//���Ʒ��οɽ��б任
					points[j].x = x + NORMDOUBLE(0, positionError);
					points[j].y = y + NORMDOUBLE(0, positionError);
				}
				else {
					points[j].x = x;
					points[j].y = y;
				}
			}
			Building2D* building = new Building2D();
			std::vector<Segment2D*> segments(points.size() - 1);					/** @brief	��ȡ������	*/
			for (int j = 0; j < points.size() - 1; ++j) {
				Segment2D* segment = new Segment2D(points[j], points[j + 1], BUILDING2D);
				segments[j] = segment;
			}
			building->SetSegments(segments);
			building->m_buildingId = buildingId++;
			outBuildings.push_back(building);
		}
	}


	//��ȡ�����ļ�����
	std::stringstream ass;
	ass << astream.rdbuf();
	astream.close();
	std::vector<std::string> alines;
	std::string aline;
	while (std::getline(ass, aline)) {
		if (aline.empty())
			continue;
		alines.push_back(aline);
	}
	if (alines.size() != outBuildings.size()) {
		LOG_ERROR << "Building2D: incompatible attribute size." << ENDL;
		return false;
	}
	//����boost���ȡ��Ӧ���ı�����
	for (int i = 0; i < alines.size(); ++i) {
		std::vector<std::string> cols;
		boost::split(cols, alines[i], boost::is_any_of(" \t,"));
		if (cols.size() < 9) {
			LOG_ERROR << "Building2D: attribute format wrong, line " << i << " ." << ENDL;
			return false;
		}
		RtLbsType height = boost::lexical_cast<RtLbsType>(cols[2]);
		int matId = boost::lexical_cast<int>(cols[3]);
		bool hasReflection = boost::lexical_cast<bool>(cols[4]);
		bool hasDiffraction = boost::lexical_cast<bool>(cols[5]);
		bool hasTransmission = boost::lexical_cast<bool>(cols[6]);
		bool hasEmpiricalTransmission = boost::lexical_cast<bool>(cols[7]);
		bool hasScattering = boost::lexical_cast<bool>(cols[8]);
		if (hasDiffraction)														//�����������ԣ���Ѱ�������
			outBuildings[i]->InitWedges();
		outBuildings[i]->m_height = height;																										//�߶ȸ�ֵ
		outBuildings[i]->m_matId = matId;																										//����ID
		outBuildings[i]->m_propagationProperty.Set(hasReflection, hasDiffraction, hasTransmission, hasEmpiricalTransmission, hasScattering);	//�������Ը�ֵ

		
	}

	LOG_INFO << "Building2D: loading building success." << ENDL;
	return true;
}
