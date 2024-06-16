#ifndef RTLBS_ANGULARELEMENT
#define RTLBS_ANGULARELEMENT

#include "rtlbs.h"
#include "utility/define.h"

class AngularElement {
public:
	bool m_isValid;							/** @brief	是否有效	*/
	int m_category;							/** @brief	角元种类	*/
	RtLbsType m_power;						/** @brief	角元上功率	*/
	RtLbsType m_startTheta;					/** @brief	起始角度值	*/


public:
	AngularElement();
	AngularElement(const AngularElement& element);
	~AngularElement();
	AngularElement& operator = (const AngularElement& element);
};

#endif
