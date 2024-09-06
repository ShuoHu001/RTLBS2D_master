#include "elevationmatrix.h"

ElevationMatrix::ElevationMatrix()
	: m_xNum(0)
	, m_yNum(0)
	, m_xMin(FLT_MAX)
	, m_xMax(-FLT_MAX)
	, m_yMin(FLT_MAX)
	, m_yMax(-FLT_MAX)
	, m_gap(0.0)
{
}

ElevationMatrix::~ElevationMatrix()
{
}

bool ElevationMatrix::Init(const std::string& filename, int mode)
{
	struct ElevationData 
	{
		Point2D position;
		RtLbsType value;
	};
	std::ifstream stream(filename.c_str());
	if (!stream.is_open()) {
		LOG_ERROR << "ErrorMatrix: failed to load " << filename << " ." << ENDL;
		stream.close();
		return false;
	}
	
	std::string line;
	if (mode == 0) {												//0模式表示 矩阵形式读取
		if (getline(stream, line)) {
			std::vector<std::string> cols;
			if (cols.size() == 8) {//读取头数据
				boost::split(cols, line, boost::is_any_of(" \t,"));
				m_xMin = boost::lexical_cast<RtLbsType>(cols[1]);
				m_xMax = boost::lexical_cast<RtLbsType>(cols[2]);
				m_yMin = boost::lexical_cast<RtLbsType>(cols[3]);
				m_yMax = boost::lexical_cast<RtLbsType>(cols[4]);
				m_gap = boost::lexical_cast<RtLbsType>(cols[5]);
				m_xNum = boost::lexical_cast<int>(cols[6]);
				m_yNum = boost::lexical_cast<int>(cols[7]);
				m_data.resize(m_xNum * m_yNum);
				for (int j = 0; j < m_yNum; ++j) {//读入顺序-先Y后X
					if (getline(stream, line)) {
						LOG_ERROR << "ErrorMatrix: wrong file format." << ENDL;
						stream.close();
						return false;
					}
					boost::split(cols, line, boost::is_any_of(" \t,"));
					if (cols.size() != m_xNum) {
						LOG_ERROR << "ErrorMatrix: wrong file format." << ENDL;
						stream.close();
						return false;
					}
					for (int i = 0; i < m_xNum; ++i) {
						int offset = j * m_xNum + i;
						RtLbsType value = boost::lexical_cast<RtLbsType>(cols[offset]);
						m_data[offset] = value;
					}
				}
				stream.close();
				return true;
			}
		}
	}
	else if (mode == 1) {												//1模式表示序列点形式读取
		//读取所有行数据
		std::vector<ElevationData> datas;
		while (getline(stream, line)) {
			std::vector<std::string> cols;
			boost::split(cols, line, boost::is_any_of(" \t,"));
			if (cols.size() != 3) {
				LOG_ERROR << "ErrorMatrix: wrong file format." << ENDL;
				stream.close();
				return false;
			}
			ElevationData curData;
			curData.position.x = boost::lexical_cast<RtLbsType>(cols[0]);
			curData.position.y = boost::lexical_cast<RtLbsType>(cols[1]);
			curData.value = boost::lexical_cast<RtLbsType>(cols[2]);
			datas.push_back(curData);
		}
		stream.close();

		//计算间隔
		m_gap = datas[1].position.x - datas[0].position.x;

		//计算最大最小值
		for (auto curData : datas) {
			m_xMin = std::min(curData.position.x, m_xMin);
			m_xMax = std::max(curData.position.x, m_xMax);
			m_yMin = std::min(curData.position.y, m_yMin);
			m_yMax = std::max(curData.position.y, m_yMax);
		}

		//计算数量
		m_xNum = static_cast<int>(std::round((m_xMax - m_xMin) / m_gap)) + 1;
		m_yNum = static_cast<int>(std::round((m_yMax - m_yMin) / m_gap)) + 1;

		m_data.resize(m_xNum* m_yNum);

		for (int i = 0; i < datas.size(); ++i) {
			m_data[i] = datas[i].value;
		}
		return true;
	}
	
	stream.close();
	LOG_ERROR << "ErrorMatrix: wrong file format." << ENDL;
	return false;
}

RtLbsType ElevationMatrix::GetValue(const Point2D& p) const
{
	if (p.x < m_xMin || p.x > m_yMax || p.y < m_yMin || p.y > m_yMax) {
		//LOG_WARNING << "ElevationMatrix: position out of boundary." << ENDL;
		return 0.5;
	}
	int xOffset = static_cast<int>(std::round((p.x - m_xMin) / m_gap));
	int yOffset = static_cast<int>(std::round((p.y - m_yMin) / m_gap));
	int offset = yOffset * m_xNum + xOffset;
	return m_data[offset];
}

RtLbsType ElevationMatrix::GetValue(const Point3D& p) const
{
	if (p.x < m_xMin || p.x > m_yMax || p.y < m_yMin || p.y > m_yMax) {
		LOG_WARNING << "ElevationMatrix: position out of boundary." << ENDL;
		return 0.5;
	}
	int xOffset = static_cast<int>(std::round((p.x - m_xMin) / m_gap));
	int yOffset = static_cast<int>(std::round((p.y - m_yMin) / m_gap));
	int offset = yOffset * m_xNum + xOffset;
	return m_data[offset];
}
