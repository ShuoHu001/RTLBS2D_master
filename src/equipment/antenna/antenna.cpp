#include "antenna.h"

Antenna::Antenna()
	: m_antId(0)
	, m_typeId(0)
	, m_gain(0)
	, m_freqMin(FLT_MAX)
	, m_freqMax(-FLT_MAX)
{
}

Antenna::Antenna(const Antenna& ant)
	: m_antId(ant.m_antId)
	, m_typeId(ant.m_typeId)
	, m_antName(ant.m_antName)
	, m_gain(ant.m_gain)
	, m_freqMin(ant.m_freqMin)
	, m_freqMax(ant.m_freqMax)
	, m_polarization(ant.m_polarization)
	, m_posture(ant.m_posture)
	, m_patterns(ant.m_patterns)
{
}

Antenna::Antenna(const AntennaConfig& config)
	: m_antId(config.m_antId)
	, m_typeId(config.m_typeId)
	, m_antName(config.m_antName)
	, m_gain(config.m_typicalGain)
	, m_freqMin(config.m_freqMin)
	, m_freqMax(config.m_freqMax)
	, m_polarization(config.m_polarization)
	, m_posture(config.m_posture)
{
	//���ļ��ж�ȡ���߷���ͼ
}

Antenna::Antenna(int innerAntId)
{
	if (innerAntId == 0) {
		m_antId = 0;
		m_typeId = 0;
		m_antName = "0-Omni";
		m_gain = 0;
		m_freqMin = 100e6;
		m_freqMax = 100e9;
		m_polarization = Vector3D(0, 0, 1, true);
	}
	else if (innerAntId == 1) {
		m_antId = 1;
		m_typeId = 1;
		m_antName = "1-full-dipole";
		m_gain = 1.2589;
		m_freqMin = 100e6;
		m_freqMax = 100e9;
		m_polarization = Vector3D(0, 0, 1, true);
	}
	else if (innerAntId == 2) {
		m_antId = 2;
		m_typeId = 2;
		m_antName = "2-half-dipole";
		m_gain = 1.2589;
		m_freqMin = 100e6;
		m_freqMax = 100e9;
		m_polarization = Vector3D(0, 0, 1, true);
	}
	else if (innerAntId == 3) {
		m_antId = 3;
		m_typeId = 3;
		m_antName = "3-three-half-dipole";
		m_gain = 1.2589;
		m_freqMin = 100e6;
		m_freqMax = 100e9;
		m_polarization = Vector3D(0, 0, 1, true);
	}
	else if (innerAntId == 4) {
		m_antId = 4;
		m_typeId = 4;
		m_antName = "4-single-wound-helical";
		m_gain = 1.2589;
		m_freqMin = 100e6;
		m_freqMax = 100e9;
		m_polarization = Vector3D(0, 0, 1, true);
	}
	else {				//Ĭ�ϲ���id==1Ϊȫ������
		m_antId = 0;
		m_typeId = 0;
		m_antName = "0-Omni";
		m_gain = 0;
		m_freqMin = 100e6;
		m_freqMax = 100e9;
		m_polarization = Vector3D(0, 0, 1, true);
	}
}

Antenna::~Antenna()
{
}

Antenna& Antenna::operator=(const Antenna& ant)
{
	m_typeId = ant.m_typeId;
	m_antName = ant.m_antName;
	m_gain = ant.m_gain;
	m_freqMin = ant.m_freqMin;
	m_freqMax = ant.m_freqMax;
	m_polarization = ant.m_polarization;
	m_posture = ant.m_posture;
	m_patterns = ant.m_patterns;
	return *this;
	// TODO: �ڴ˴����� return ���
}

bool Antenna::Init(std::string filename)
{
	LOG_INFO << "start reading file: " << filename << ENDL;
	std::ifstream config_ifs(filename);
	if (!config_ifs.is_open()) {
		LOG_ERROR << filename << ": not exist" << ENDL;
		this->Write2Json(filename);
		LOG_WARNING << "system has wrote default configuration to " << filename << " please check and modify." << CRASH;
		return false;
	}
	std::stringstream ss;
	ss << config_ifs.rdbuf();
	config_ifs.close();
	std::string jsonString = ss.str();
	rapidjson::Document doc;
	doc.Parse(jsonString.c_str());
	rapidjson::Value& value = doc["AntennaConfig"];
	if (value.IsObject()) {
		if (Deserialize(value)) {
			LOG_INFO << "loading antenna file success" << ENDL;
			return true;
		}
	}
	LOG_ERROR << "loading configuration file failed, please check file format." << CRASH;
	return false;
}

void Antenna::Write2Json(std::string filename)
{
	std::ofstream config_ofs(filename);
	if (!config_ofs.is_open()) {
		LOG_ERROR << filename << ": file not exist" << CRASH;
		return;
	}
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
	writer.Key("AntennaConfig"); this->Serialize(writer);
	writer.EndObject();
	config_ofs << sb.GetString();
	config_ofs.close();
	return;
}


RtLbsType Antenna::GetAntennaNormalizedGain(RtLbsType freq, RtLbsType phi, RtLbsType theta) const
{
	double gain = 1.0;
	if (m_typeId >= 0 && m_typeId <= 99)															//0-99Ϊ���ñ��
		gain = _calcInternalAntennaGain(m_typeId, phi, theta);
	else {																						//100-Ϊ�������߱��
		gain = _calcExternalAntennaGain(m_typeId, freq, phi, theta);
	}
	return gain;
}

void Antenna::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
	writer.StartObject();
	writer.Key(KEY_ANTENNA_TYPEID.c_str()); writer.Int(m_typeId);
	writer.Key(KEY_ANTENNA_ANTNAME.c_str()); writer.String(m_antName.c_str());
	writer.Key(KEY_ANTENNA_GAIN.c_str()); writer.Double(m_gain);
	writer.Key(KEY_ANTENNA_FREQMIN.c_str()); writer.Double(m_freqMin);
	writer.Key(KEY_ANTENNA_FREQMAX.c_str()); writer.Double(m_freqMax);
	writer.Key(KEY_ANTENNA_POLARIZATION.c_str()); m_polarization.Serialize(writer);
	writer.Key(KEY_ANTENNA_POSTURE.c_str()); m_posture.Serialize(writer);
	writer.EndObject();
}

bool Antenna::Deserialize(const rapidjson::Value& value)
{
	if (value.IsObject()) {
		const rapidjson::Value& typeIdValue = value[KEY_ANTENNA_TYPEID.c_str()];
		const rapidjson::Value& antNameValue = value[KEY_ANTENNA_ANTNAME.c_str()];
		const rapidjson::Value& gainValue = value[KEY_ANTENNA_GAIN.c_str()];
		const rapidjson::Value& freqMinValue = value[KEY_ANTENNA_FREQMIN.c_str()];
		const rapidjson::Value& freqMaxValue = value[KEY_ANTENNA_FREQMAX.c_str()];
		const rapidjson::Value& polarizationValue = value[KEY_ANTENNA_POLARIZATION.c_str()];
		const rapidjson::Value& postureValue = value[KEY_ANTENNA_POSTURE.c_str()];
		if (value.HasMember(KEY_ANTENNA_TYPEID.c_str()) &&
			value.HasMember(KEY_ANTENNA_ANTNAME.c_str()) &&
			value.HasMember(KEY_ANTENNA_GAIN.c_str()) &&
			value.HasMember(KEY_ANTENNA_FREQMIN.c_str()) &&
			value.HasMember(KEY_ANTENNA_FREQMAX.c_str()) &&
			value.HasMember(KEY_ANTENNA_POLARIZATION.c_str()) &&
			value.HasMember(KEY_ANTENNA_POSTURE.c_str())) {
			if (typeIdValue.IsInt() &&
				antNameValue.IsString() &&
				gainValue.IsDouble() &&
				freqMinValue.IsDouble() &&
				freqMaxValue.IsDouble() &&
				polarizationValue.IsObject() &&
				postureValue.IsObject()) {
				m_typeId = typeIdValue.GetInt();
				m_antName = antNameValue.GetString();
				m_gain = gainValue.GetDouble();
				m_freqMin = freqMinValue.GetDouble();
				m_freqMax = freqMaxValue.GetDouble();
				if (m_polarization.Deserialize(polarizationValue) &&
					m_posture.Deserialize(postureValue)) {
					return true;
				}
			}
		}
	}
	return false;
}

double Antenna::_calcInternalAntennaGain(int id, double azimuth, double elevation) const
{
	double gain = 0;
	switch (id) {
	case 0:													//0-ȫ������
		gain = _calcOmniAntennaGain();
		break;
	case 1:													//1-ȫ��ż��������
		gain = _calcWaveDipoleAntennaGain(elevation);
		break;
	case 2:													//2-�벨ż��������
		gain = _calcHalfWaveDipoleAntennaGain(elevation);
		break;
	case 3:													//3-�����벨ż��������
		gain = _calcThreeHalfWaveDipoleAntennaGain(elevation);
		break;
	case 4:													//4-���ƻ�������
		gain = _calcSingleWoundSpiralAntennaGain(azimuth, elevation, 1);
		break;
	default:												//Ĭ��ȫ�����ߵ����߷���ͼ����
		break;
	}
	
	return gain;
}

double Antenna::_calcExternalAntennaGain(int id, RtLbsType freq, RtLbsType azimuth, RtLbsType elevation) const
{
	//��Ѱ�ҵ�Ƶ����������߷���ͼ��Ϊƥ������߷���ͼ
	if (m_patterns.size() == 0)															//�����߷���ͼ�������ݣ���Ĭ�ϸ���ȫ�����߷���ͼ
		return 1.0;
	RtLbsType freqDistanceMin = abs(freq - m_patterns[0].m_freq);
	int minId = 0;
	for (int i = 1; i < m_patterns.size(); ++i) {
		const AntennaPattern& pattern = m_patterns[i];
		RtLbsType freqDistance = abs(freq - pattern.m_freq);
		if (freqDistance < freqDistanceMin) {
			minId = i;
		}
	}
	double gain = m_patterns[minId].GetGain(azimuth, elevation);
	return gain;
}

double Antenna::_calcOmniAntennaGain() const
{
	return 1.0;
}

double Antenna::_calcWaveDipoleAntennaGain(double elevation) const
{
	double gain = cos(HALF_PI * cos(elevation)) / sin(elevation);
	return gain;
}

double Antenna::_calcHalfWaveDipoleAntennaGain(double elevation) const
{
	double gain = (cos(PI * cos(elevation)) + 1) / sin(elevation);
	return gain;
}

double Antenna::_calcThreeHalfWaveDipoleAntennaGain(double elevation) const
{
	double gain = cos(3.0 * HALF_PI * cos(elevation)) / sin(elevation);
	return gain;
}

double Antenna::_calcSingleWoundSpiralAntennaGain(double azimuth, double elevation, double n) const
{
	double gain = sin(90.0 / n / PI) * sin(n * azimuth / 2) / sin(azimuth / 2) * cos(elevation);
	return gain;
}

