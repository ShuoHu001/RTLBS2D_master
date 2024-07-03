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
	int m_antId;									/** @brief	天线ID	*/
	int m_typeId;									/** @brief	天线类型Id 0-100(不含)为内置天线，100-为自定义天线	*/
	std::string m_antName;							/** @brief	天线名称	*/
	RtLbsType m_gain;								/** @brief	天线增益	*/
	RtLbsType m_freqMin;							/** @brief	频率最小值/Hz	*/
	RtLbsType m_freqMax;							/** @brief	频率最大值/Hz	*/
	Vector3D m_polarization;						/** @brief	极化方式	*/
	Euler m_posture;								/** @brief	三维姿态	*/

private:
	std::vector<AntennaPattern> m_patterns;			/** @brief	天线方向图数据（按照频率分）,暂时不考虑	*/
	

public:
	Antenna();
	Antenna(const Antenna& ant);
	Antenna(const AntennaConfig& config);
	~Antenna();
	Antenna& operator = (const Antenna& ant);
	bool Init(std::string filename);																												//初始化天线参数
	void Write2Json(std::string filename);																											//将序列化结果写入文件中
	RtLbsType GetAntennaNormalizedGain(RtLbsType freq, RtLbsType phi, RtLbsType theta) const;														//获取任意方向的天线方向归一化增益值
	void CalRadiationField_ReverseRT(RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, RtLbsType distance, Polarization3D& efield) const;		//计算天线周围的辐射场
	void CalRadiationField_ForwardRT(RtLbsType power, RtLbsType freq, RtLbsType phi, RtLbsType theta, Polarization3D& efield) const;							//计算天线周围的辐射场-正向电磁计算（与距离无关）
	void CalReceivedField(Polarization3D& eIn, RtLbsType freq, RtLbsType phi, RtLbsType theta, Complex& eOut) const;								//计算天线周围接收场
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);																		//序列化对象
	bool Deserialize(const rapidjson::Value& value);																								//反序列化对象

private:
	double _calcInternalAntennaGain(int id, double azimuth, double elevation) const;																//计算内置天线的方向图函数
	double _calcExternalAntennaGain(int id, RtLbsType freq, RtLbsType azimuth, RtLbsType elevation) const;											//计算外置天线的天线方向图函数
	double _calcOmniAntennaGain() const;																											//计算全向天线的方向图函数
	double _calcWaveDipoleAntennaGain(double elevation) const;																						//计算全波偶极子的方向图函数
	double _calcHalfWaveDipoleAntennaGain(double elevation) const;																					//计算半波偶极子的方向图函数
	double _calcThreeHalfWaveDipoleAntennaGain(double elevation) const;																				//计算三倍半波偶极子的方向图函数
	double _calcSingleWoundSpiralAntennaGain(double azimuth, double elevation, double n) const;														//计算单绕螺旋天线的方向图函数,n 为环绕圈数

};



#endif
