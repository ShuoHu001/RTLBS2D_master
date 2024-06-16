#ifndef RTLBS_PATTERN
#define RTLBS_PATTERN

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"

class AntennaPattern:public Serializable {

public:
	int m_patternId;					/** @brief	天线方向图 Id	*/
	ANTENNAPATTERN_TYPE m_patternType;	/** @brief	天线方向图类型 2D|3D	*/
	RtLbsType m_freq;					/** @brief	天线方向图 频率	*/
private:
	RtLbsType m_eInterval2D;			/** @brief	E 面数据间隔 二维	*/
	RtLbsType m_hInterval2D;			/** @brief	H 面数据间隔 二维	*/
	std::vector<RtLbsType> m_eData;			/** @brief	E 面天线方向图	*/
	std::vector<RtLbsType> m_hData;			/** @brief	H 面天线方向图 	*/
	RtLbsType m_eInterval3D;			/** @brief	E 面数据间隔 三维	*/
	RtLbsType m_hInterval3D;			/** @brief	H 面数据间隔 三维	*/
	std::vector<RtLbsType> m_ehData;			/** @brief	EH三维天线方向图(一维数组 e→h)	*/


public:
	AntennaPattern();
	~AntennaPattern();
	RtLbsType GetGain(RtLbsType azimuth, RtLbsType elevation) const;		//基于插值算法获取三维天线方向图增益（归一化增益值）

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};

#endif
