#ifndef RTLBS_ANGLESPECTRUM
#define RTLBS_ANGLESPECTRUM

#include "rtlbs.h"
#include "utility/define.h"
#include "angularelement.h"

using std::vector;

class AngularSpectrum {
public:
	RtLbsType m_size;							/** @brief	将2PI分为N个部分	*/
	std::vector<AngularElement> m_units;		/** @brief	AOA角谱单元	*/

public:
	AngularSpectrum();
	AngularSpectrum(const AngularSpectrum& spectrum);
	~AngularSpectrum();
	AngularSpectrum& operator = (const AngularSpectrum& spectrum);
	RtLbsType CalUnitAngle() const;
	bool InitFromFile(const std::string& filename);					//从文件中加载角度谱
	RtLbsType GetReceivedPower(RtLbsType theta);					//给定AOD值
};

#endif
