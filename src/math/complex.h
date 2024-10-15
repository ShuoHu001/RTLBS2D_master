#ifndef RTLBS_COMPLEX
#define RTLBS_COMPLEX

#include "rtlbs.h"
#include "utility/define.h"
#include "vector3d.h"

class Polarization3D;

class Complex {
public:
	RtLbsType m_real;
	RtLbsType m_imag;

public:
	Complex();
	Complex(RtLbsType real, RtLbsType imag);
	Complex(const Complex& c);
	~Complex();
	Complex& operator = (const Complex& c);				//复制构造函数
	Complex operator + (const Complex& c) const;
	Complex& operator += (const Complex& c);
	Complex operator - (const Complex& c) const;
	Complex& operator -= (const Complex& c);
	Complex operator * (const Complex& c) const;
	Complex& operator *= (const Complex& c);
	Complex operator * (const RtLbsType num) const;
	Complex& operator *= (const RtLbsType num);
	Complex operator / (const Complex& c) const;
	Complex& operator /= (const Complex& c);
	Complex operator / (const RtLbsType num) const;
	Complex& operator /= (const RtLbsType num);
	RtLbsType operator [] (unsigned id) const;
	RtLbsType& operator [] (unsigned id);
	bool operator == (const Complex& c) const;
	bool operator != (const Complex& c) const;
	Polarization3D operator * (const Vector3D& v) const;	//将常规复数扩展为三维空间场值
	void Init(RtLbsType amplitude, RtLbsType phase);		//由幅度和相位构成复数
	RtLbsType MValue() const;								//复数模值
	RtLbsType MValue2() const;								//复数模值平方
	Complex Sqrt();											//复数平方根
	Complex Reciprocal();									//复数倒数
	RtLbsType GetRad() const;								//获取复数的角度/相位
	bool IsZero() const;									//是否是零值

};

inline Complex Exp(RtLbsType y) {
	Complex c;
	c.m_real = cos(y);
	c.m_imag = sin(y);
	return c;
}

#endif
