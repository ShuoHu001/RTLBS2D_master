#ifndef RTLBS_ANTENNA
#define RTLBS_ANTENNA

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "antennapattern.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "physical/radiowave/polarization3d.h"
#include "configuration/antennaconfig.h"

const std::string KEY_ANTENNA_TYPEID = "TypeId";
const std::string KEY_ANTENNA_ANTNAME = "AntName";
const std::string KEY_ANTENNA_GAIN = "Gain";
const std::string KEY_ANTENNA_FREQMIN = "FreqMin";
const std::string KEY_ANTENNA_FREQMAX = "FreqMax";
const std::string KEY_ANTENNA_POLARIZATION = "Polarization";
const std::string KEY_ANTENNA_POSTURE = "Posture";

class Antenna:public Serializable {
public:
	int m_antId;									/** @brief	����ID	*/
	int m_typeId;									/** @brief	��������Id 0-100(����)Ϊ�������ߣ�100-Ϊ�Զ�������	*/
	std::string m_antName;							/** @brief	��������	*/
	RtLbsType m_gain;								/** @brief	��������	*/
	RtLbsType m_freqMin;							/** @brief	Ƶ����Сֵ/Hz	*/
	RtLbsType m_freqMax;							/** @brief	Ƶ�����ֵ/Hz	*/
	Vector3D m_polarization;						/** @brief	������ʽ	*/
	Euler m_posture;								/** @brief	��ά��̬	*/

private:
	std::vector<AntennaPattern> m_patterns;			/** @brief	���߷���ͼ���ݣ�����Ƶ�ʷ֣�,��ʱ������	*/
	

public:
	Antenna();
	Antenna(const Antenna& ant);
	Antenna(const AntennaConfig& config);
	~Antenna();
	Antenna& operator = (const Antenna& ant);
	bool Init(std::string filename);																												//��ʼ�����߲���
	void Write2Json(std::string filename);																											//�����л����д���ļ���
	RtLbsType GetAntennaNormalizedGain(RtLbsType freq, RtLbsType phi, RtLbsType theta) const;														//��ȡ���ⷽ������߷����һ������ֵ
	void CalRadiationField_ReverseRT(RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, RtLbsType distance, Polarization3D& efield) const;		//����������Χ�ķ��䳡
	void CalRadiationField_ForwardRT(RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, Polarization3D& efield) const;							//����������Χ�ķ��䳡-�����ż��㣨������޹أ�
	void CalReceivedField(Polarization3D& eIn, RtLbsType freq, RtLbsType phi, RtLbsType theta, Complex& eOut) const;								//����������Χ���ճ�
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);																		//���л�����
	bool Deserialize(const rapidjson::Value& value);																								//�����л�����

private:
	double _calcInternalAntennaGain(int id, double azimuth, double elevation) const;																//�����������ߵķ���ͼ����
	double _calcExternalAntennaGain(int id, RtLbsType freq, RtLbsType azimuth, RtLbsType elevation) const;											//�����������ߵ����߷���ͼ����
	double _calcOmniAntennaGain() const;																											//����ȫ�����ߵķ���ͼ����
	double _calcWaveDipoleAntennaGain(double elevation) const;																						//����ȫ��ż���ӵķ���ͼ����
	double _calcHalfWaveDipoleAntennaGain(double elevation) const;																					//����벨ż���ӵķ���ͼ����
	double _calcThreeHalfWaveDipoleAntennaGain(double elevation) const;																				//���������벨ż���ӵķ���ͼ����
	double _calcSingleWoundSpiralAntennaGain(double azimuth, double elevation, double n) const;														//���㵥���������ߵķ���ͼ����,n Ϊ����Ȧ��

};



#endif
