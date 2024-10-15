#ifndef RTLBS_POLARIZATION2D
#define RTLBS_POLARIZATION2D

#include "rtlbs.h"
#include "utility/define.h"
#include "math/complex.h"

//二维极化电场（垂直极化、水平极化）
class Polarization2D {
public:
	Complex perp;			//垂直极化方向分量
	Complex para;			//平行极化方向分量

public:
	Polarization2D();
	Polarization2D(Complex perpE, Complex paraE);
	Polarization2D(Polarization2D& p);
	~Polarization2D();
	Polarization2D& operator = (const Polarization2D& p);
	Polarization2D operator + (const Polarization2D& p) const;
	Polarization2D& operator += (const Polarization2D& p);
	Polarization2D operator - (const Polarization2D& p) const;
	Polarization2D& operator -= (const Polarization2D& p);
	Polarization2D operator * (const RtLbsType num) const;
	Polarization2D& operator *= (const RtLbsType num);
	Polarization2D operator / (const RtLbsType num) const;
	Polarization2D& operator /= (const RtLbsType num);
	Complex operator [] (const unsigned id) const;
	Complex& operator [] (const unsigned id);
};

#endif
