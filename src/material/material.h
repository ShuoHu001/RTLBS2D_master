#ifndef RTLBS_MATERIAL
#define RTLBS_MATERIAL

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "math/complex.h"
#include "configuration/materialconfig.h"


class Material {
public:
	int m_id;							/** @brief	����ID	*/
	std::string m_name;					/** @brief	��������	*/
	RtLbsType m_permittivity;			/** @brief	������Խ�糣��	*/
	RtLbsType m_conductivity;			/** @brief	���ϵ絼��	*/
	RtLbsType m_penetrationLoss;		/** @brief	�����ڲ��Ĳ�����ģ�ͳ��ֵ	*/
	RtLbsType m_refractiveN;			/** @brief	����������	*/

public:
	Material();
	Material(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity);
	Material(int matId, const std::string& name, RtLbsType permittivity, RtLbsType conductivity, RtLbsType penetractionLoss);
	Material(const Material& mat);
	Material(const MaterialConfig& config);
	~Material();
	Material& operator = (const Material& mat);
	bool operator == (const Material& mat) const;
	bool operator != (const Material& mat) const;
	Complex GetComplexForm(RtLbsType freq) const;			//��ò��ʵĸ�����ʽ
	RtLbsType GetRefractiveN() const;						//��ò��ʵ�������
};

#endif
