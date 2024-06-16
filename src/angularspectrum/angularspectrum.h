#ifndef RTLBS_ANGLESPECTRUM
#define RTLBS_ANGLESPECTRUM

#include "rtlbs.h"
#include "utility/define.h"
#include "angularelement.h"

using std::vector;

class AngularSpectrum {
public:
	RtLbsType m_size;							/** @brief	��2PI��ΪN������	*/
	std::vector<AngularElement> m_units;		/** @brief	AOA���׵�Ԫ	*/

public:
	AngularSpectrum();
	AngularSpectrum(const AngularSpectrum& spectrum);
	~AngularSpectrum();
	AngularSpectrum& operator = (const AngularSpectrum& spectrum);
	RtLbsType CalUnitAngle() const;
	bool InitFromFile(const std::string& filename);					//���ļ��м��ؽǶ���
	RtLbsType GetReceivedPower(RtLbsType theta);					//����AODֵ
};

#endif
