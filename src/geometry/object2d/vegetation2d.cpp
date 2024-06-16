#include "vegetation2d.h"

Vegetation2D::Vegetation2D()
	: Object2D()
	, m_vegetationId(-1)
{
}

Vegetation2D::~Vegetation2D()
{
}

bool LoadVegetationsFromFile(const std::string& geometryFile, const std::string& attributeFile, std::vector<Vegetation2D*>& outVegetations)
{

	if (geometryFile.empty() || attributeFile.empty()) {
		LOG_ERROR << "Vegetation2D: empty filename." << ENDL;
		return false;
	}
	std::ifstream gstream(geometryFile);					/** @brief	几何文件读取流	*/
	if (!gstream.is_open()) {
		LOG_ERROR << "Vegetation2D: failed to open " << geometryFile << " ." << ENDL;
		return false;
	}

	std::ifstream astream(attributeFile);					/** @brief	属性文件读取流	*/
	if (!astream.is_open()) {
		LOG_ERROR << "Vegetation2D: failed to open " << attributeFile << " ." << ENDL;
		return false;
	}

	//读取几何文件数据
	std::stringstream gss;
	gss << gstream.rdbuf();
	gstream.close();
	std::vector<std::string> glines;
	std::string gline;
	while (std::getline(gss, gline)) {
		glines.push_back(gline);
	}
	int segmentId = 0;											/** @brief	线段内部ID	*/
	int vegetationId = 0;
	std::vector<Point2D> points;								/** @brief	读取的点坐标	*/
	for (int i = 0; i < glines.size(); ++i) {
		std::vector<std::string> cols;
		boost::split(cols, glines[i], boost::is_any_of(" \t,"));
		if (cols.size() == 3) {			//building 识别文件
			Vegetation2D* vegetation = new Vegetation2D();
			std::vector<Segment2D*> segments;					/** @brief	读取的形体	*/
			for (int j = 0; j < points.size() - 1; ++j) {
				Segment2D* segment = new Segment2D(points[i], points[i + 1], segmentId++, vegetationId, VEGETATION2D);
			}
			vegetation->SetSegments(segments);
			vegetation->m_vegetationId = vegetationId++;
			outVegetations.push_back(vegetation);
			points.clear();
		}
		else {
			RtLbsType x = boost::lexical_cast<RtLbsType>(cols[0]);
			RtLbsType y = boost::lexical_cast<RtLbsType>(cols[1]);
			points.push_back({ x,y });
		}
	}

	//读取属性文件数据
	std::stringstream ass;
	ass << astream.rdbuf();
	astream.close();
	std::vector<std::string> alines;
	std::string aline;
	while (std::getline(ass, aline)) {
		alines.push_back(aline);
	}
	if (alines.size() != outVegetations.size()) {
		LOG_ERROR << "Vegetation2D: incompatible attribute size." << ENDL;
		return false;
	}
	//基于boost库读取对应的文本数据
	for (int i = 0; i < alines.size(); ++i) {
		std::vector<std::string> cols;
		boost::split(cols, alines[i], boost::is_any_of(" \t,"));
		if (cols.size() < 9) {
			LOG_ERROR << "Vegetation2D: attribute format wrong, line " << i << " ." << ENDL;
			return false;
		}
		RtLbsType height = boost::lexical_cast<RtLbsType>(cols[2]);
		int matId = boost::lexical_cast<int>(cols[3]);
		bool hasReflection = boost::lexical_cast<bool>(cols[4]);
		bool hasDiffraction = boost::lexical_cast<bool>(cols[5]);
		bool hasTransmission = boost::lexical_cast<bool>(cols[6]);
		bool hasEmpiricalTransmission = boost::lexical_cast<bool>(cols[7]);
		bool hasScattering = boost::lexical_cast<bool>(cols[8]);
		hasDiffraction = false;																													//默认材质无绕射属性
		outVegetations[i]->m_height = height;																									//高度赋值
		outVegetations[i]->m_matId = matId;																										//材质ID
		outVegetations[i]->m_propagationProperty.Set(hasReflection, hasDiffraction, hasTransmission, hasEmpiricalTransmission, hasScattering);	//传播属性赋值
	}
	LOG_INFO << "Vegetation2D: loading vegetation success." << ENDL;
	return true;
}
