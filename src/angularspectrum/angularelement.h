#ifndef RTLBS_ANGULARELEMENT
#define RTLBS_ANGULARELEMENT

#include "rtlbs.h"
#include "utility/define.h"

class AngularElement {
public:
	bool m_isValid;							/** @brief	�Ƿ���Ч	*/
	int m_category;							/** @brief	��Ԫ����	*/
	RtLbsType m_power;						/** @brief	��Ԫ�Ϲ���	*/
	RtLbsType m_startTheta;					/** @brief	��ʼ�Ƕ�ֵ	*/


public:
	AngularElement();
	AngularElement(const AngularElement& element);
	~AngularElement();
	AngularElement& operator = (const AngularElement& element);
};

#endif
