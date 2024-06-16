#ifndef RTLBS_PATTERN
#define RTLBS_PATTERN

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "utility/serializable.h"

class AntennaPattern:public Serializable {

public:
	int m_patternId;					/** @brief	���߷���ͼ Id	*/
	ANTENNAPATTERN_TYPE m_patternType;	/** @brief	���߷���ͼ���� 2D|3D	*/
	RtLbsType m_freq;					/** @brief	���߷���ͼ Ƶ��	*/
private:
	RtLbsType m_eInterval2D;			/** @brief	E �����ݼ�� ��ά	*/
	RtLbsType m_hInterval2D;			/** @brief	H �����ݼ�� ��ά	*/
	std::vector<RtLbsType> m_eData;			/** @brief	E �����߷���ͼ	*/
	std::vector<RtLbsType> m_hData;			/** @brief	H �����߷���ͼ 	*/
	RtLbsType m_eInterval3D;			/** @brief	E �����ݼ�� ��ά	*/
	RtLbsType m_hInterval3D;			/** @brief	H �����ݼ�� ��ά	*/
	std::vector<RtLbsType> m_ehData;			/** @brief	EH��ά���߷���ͼ(һά���� e��h)	*/


public:
	AntennaPattern();
	~AntennaPattern();
	RtLbsType GetGain(RtLbsType azimuth, RtLbsType elevation) const;		//���ڲ�ֵ�㷨��ȡ��ά���߷���ͼ���棨��һ������ֵ��

public:
	void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer);
	bool Deserialize(const rapidjson::Value& value);

};

#endif
