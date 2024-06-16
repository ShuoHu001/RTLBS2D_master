#ifndef RTLBS_SENSORDATA
#define RTLBS_SENSORDATA

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"
#include "managers/logmanager.h"
#include "geometry/vector2d.h"

const std::string KEY_SENSORDATA_TIME = "Time";
const std::string KEY_SENSORDATA_TIMEDIFF = "TimeDiff";
const std::string KEY_SENSORDATA_PHI_DEGREE = "PhiDegree";
const std::string KEY_SENSORDATA_POWER = "Power";

class SensorData {
public:
	int m_id;								/** @brief	传感器数据编号,全局唯一	*/
	int m_sensorId;							/** @brief	传感器ID	*/
	RtLbsType m_time;						/** @brief	传感器接收到的时间, 单位 ns	*/
	RtLbsType m_timeDiff;					/** @brief	传感器接收到的时间差，单位 ns	*/
	RtLbsType m_phi;						/** @brief	传感器接收到的方位角,单位 弧度	*/
	RtLbsType m_power;						/** @brief	传感器接收到的功率，单位 dBm	*/
	RtLbsType m_phiDegree;					/** @brief	传感器接收到的方位角 单位 °	*/

public:
	SensorData();
	SensorData(const SensorData& data);
	~SensorData();
	SensorData operator = (const SensorData& data);
	bool operator < (const SensorData& data);									//重载小于符号，按照功率大小进行逆向排序
	Vector2D GetDirection() const;												//将接收到的角度转换为方向矢量
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);
};

//计算角度残差二范数
inline void CalculateResidual_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//计算角度残差平方
}

//计算时间差残差二范数
inline void CalculateResidual_TDOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_timediff)
{
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//计算时间差残差平方
}

//计算时间差残差二范数和角度残差二范数
inline void CalculateResidual_TDOA_AOA(const SensorData& d1, const SensorData& d2, RtLbsType& r_phi, RtLbsType& r_timediff)
{
	r_phi = (d1.m_phi - d2.m_phi) * (d1.m_phi - d2.m_phi);								//计算角度残差平方
	r_timediff = (d1.m_timeDiff - d2.m_timeDiff) * (d1.m_timeDiff - d2.m_timeDiff);		//计算时间差残差平方
}


const std::string KEY_SENSORDATACOLLECTION = "SensorDataCollection";
const std::string KEY_SENSORDATACOLLECTION_SENSORID = "SensorId";
const std::string KEY_SENSORDATACOLLECTION_SENSORDATAS = "SensorDatas";

class SensorDataCollection {
public:
	int m_sensorId;
	std::vector<SensorData> m_data;
	
public:
	SensorDataCollection();
	SensorDataCollection(const SensorDataCollection& collection);
	~SensorDataCollection();
	SensorDataCollection& operator = (const SensorDataCollection& collection);
	void SortByPower();																	//按照能量的大小进行排序
	void CalculateTimeDiff();															//计算时延差值
	std::vector<RtLbsType> GetPowerDiffMatrix() const;									//计算功率差矩阵
	bool Init(const std::string& filename);
	void Write2Json(const std::string& filename) const;
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const;
	bool Deserialize(const rapidjson::Value& value);
};

//计算两个传感器数据的残差二范数-AOA定位方法-单数据
inline void CalculateSensorResidual_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi)
{
	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
}

//计算两组传感器数据的残差二范数-AOA定位方法-单数据
inline void CalculateSensorCollectionResidual_AOA_SingleData(std::vector<SensorDataCollection>& c1, std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff)
{
	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateSensorResidual_AOA_SingleData(c1[i], c2[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//计算功率差残差二范数和
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}

}

//计算两个传感器数据的残差二范数-AOA定位方法-多数据
inline void CalculateSensorResidual_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_powerdiff)
{
	//1-规范化处理数据源-生成数据时按照一定的分辨率进行生成，按照角度分辨率，非无限分辨率
	//2-规范化数据源排布方式-在生成数据时按照功率自大而小的顺序进行排布

	//取c1和c2中数量较少的作为比较
	int minDataNum = static_cast<int>(std::min(c1.m_data.size(), c1.m_data.size()));
	int zoomFactor = 1;																					/** @brief	残差缩放系数，若c2中只有部分满足c1的数量，则系数减少	*/
	zoomFactor = static_cast<int>(c1.m_data.size()) - minDataNum;										//差几组数据就会放大几倍
	//计算角度残差和
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_phi;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
	r_phi *= zoomFactor;

	//计算功率差残差和
	for (int i = 1; i < minDataNum; ++i) {
		RtLbsType powerDiff1 = c1.m_data[i].m_power - c1.m_data[0].m_power;
		RtLbsType powerDiff2 = c2.m_data[i].m_power - c2.m_data[0].m_power;
		r_powerdiff += (powerDiff2 - powerDiff1) * (powerDiff2 - powerDiff1);
	}
	r_powerdiff *= zoomFactor;
}

//计算两组传感器数据的残差二范数-AOA定位算法-多数据
inline void CalculateSensorCollectionResidual_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_powerDiff)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		CalculateSensorResidual_AOA_MultiData(c1[i], c2[i], cur_r_phi, cur_r_powerDiff);
		r_phi += cur_r_phi;
		r_powerDiff += cur_r_powerDiff;
	}
}


//计算两个传感器数据的残差二范数-TDOA定位方法-单数据
inline void CalculateSensorResidual_TDOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff)
{
	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}


//计算两组传感器数据的残差二范数-TDOA定位方法-单数据
inline void CalculateSensorCollectionResidual_TDOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_SingleData(c1[i], c2[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//计算功率差残差二范数和
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}


//计算两个传感器数据的残差二范数-TDOA定位方法-多数据
inline void CalculateSensorResidual_TDOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff)
{
	//取c1和c2中数量较少的作为比较
	int minDataNum = static_cast<int>(std::min(c1.m_data.size(), c1.m_data.size()));
	int zoomFactor = 1;																					/** @brief	残差缩放系数，若c2中只有部分满足c1的数量，则系数减少	*/
	zoomFactor = static_cast<int>(c1.m_data.size()) - minDataNum;										//差几组数据就会放大几倍

	//计算时延差残差二范数和
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
	r_timeDiff *= zoomFactor;

	//计算功率差残差二范数和
	for (int i = 1; i < minDataNum; ++i) {
		RtLbsType powerDiff1 = c1.m_data[i].m_power - c1.m_data[0].m_power;
		RtLbsType powerDiff2 = c2.m_data[i].m_power - c2.m_data[0].m_power;
		r_powerdiff += (powerDiff2 - powerDiff1) * (powerDiff2 - powerDiff1);
	}
	r_powerdiff *= zoomFactor;
}

//计算两组传感器数据的残差二范数-TDOA定位方法-多数据
inline void CalculateSensorCollectionResidual_TDOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		CalculateSensorResidual_TDOA_MultiData(c1[i], c2[i], cur_r_timeDiff, cur_r_powerDiff);
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
	}
}


//计算两个传感器数据的残差二范数-TDOA/AOA定位方法-单数据
inline void CalculateSensorResidual_TDOA_AOA_SingleData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff)
{
	//计算角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}

	//计算时延差残差二范数和
	for (int i = 0; i < static_cast<int>(c1.m_data.size()); ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
}


//计算两组传感器数据的残差二范数-TDOA/AOA定位方法-单数据
inline void CalculateSensorCollectionResidual_TDOA_AOA_SingleData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	//计算时延差、角度残差二范数和
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0;
		RtLbsType cur_r_timeDiff = 0;
		CalculateSensorResidual_TDOA_AOA_SingleData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}

	//计算功率差残差二范数和
	for (int i = 1; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType powerDiff1 = c1[i].m_data[0].m_power - c1[0].m_data[0].m_power;
		RtLbsType powerDiff2 = c2[i].m_data[0].m_power - c2[0].m_data[0].m_power;
		r_powerDiff += (powerDiff1 - powerDiff2) * (powerDiff1 - powerDiff2);
	}
}

//计算两个传感器数据的残差二范数-TDOA/AOA定位方法-多数据
inline void CalculateSensorResidual_TDOA_AOA_MultiData(const SensorDataCollection& c1, const SensorDataCollection& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerdiff)
{
	//取c1和c2中数量较少的作为比较
	int minDataNum = static_cast<int>(std::min(c1.m_data.size(), c1.m_data.size()));
	int zoomFactor = 1;																					/** @brief	残差缩放系数，若c2中只有部分满足c1的数量，则系数减少	*/
	zoomFactor = static_cast<int>(c1.m_data.size()) - minDataNum;										//差几组数据就会放大几倍

	//计算角度残差二范数和
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_phi = 0;
		CalculateResidual_AOA(c1.m_data[i], c2.m_data[i], cur_r_phi);
		r_phi += cur_r_phi;
	}
	r_phi *= zoomFactor;

	//计算时延差残差二范数和
	for (int i = 0; i < minDataNum; ++i) {
		RtLbsType cur_r_timeDiff = 0;
		CalculateResidual_TDOA(c1.m_data[i], c2.m_data[i], cur_r_timeDiff);
		r_timeDiff += cur_r_timeDiff;
	}
	r_timeDiff *= zoomFactor;

	//计算功率差残差二范数和
	for (int i = 1; i < minDataNum; ++i) {
		RtLbsType powerDiff1 = c1.m_data[i].m_power - c1.m_data[0].m_power;
		RtLbsType powerDiff2 = c2.m_data[i].m_power - c2.m_data[0].m_power;
		r_powerdiff += (powerDiff2 - powerDiff1) * (powerDiff2 - powerDiff1);
	}
	r_powerdiff *= zoomFactor;
}

//计算两组传感器数据的残差二范数-TDOA/AOA定位方法-多数据
inline void CalculateSensorCollectionResidual_TDOA_AOA_MultiData(const std::vector<SensorDataCollection>& c1, const std::vector<SensorDataCollection>& c2, RtLbsType& r_phi, RtLbsType& r_timeDiff, RtLbsType& r_powerDiff)
{
	for (int i = 0; i < static_cast<int>(c1.size()); ++i) {
		RtLbsType cur_r_phi = 0.0;
		RtLbsType cur_r_timeDiff = 0.0;
		RtLbsType cur_r_powerDiff = 0.0;
		CalculateSensorResidual_TDOA_AOA_MultiData(c1[i], c2[i], cur_r_phi, cur_r_timeDiff, cur_r_powerDiff);
		r_timeDiff += cur_r_timeDiff;
		r_powerDiff += cur_r_powerDiff;
	}
}





#endif
