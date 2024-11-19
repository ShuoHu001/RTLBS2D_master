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
	Complex& operator = (const Complex& c);				//���ƹ��캯��
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
	Polarization3D operator * (const Vector3D& v) const;	//�����渴����չΪ��ά�ռ䳡ֵ
	void Init(RtLbsType amplitude, RtLbsType phase);		//�ɷ��Ⱥ���λ���ɸ���
	RtLbsType MValue() const;								//����ģֵ
	RtLbsType MValue2() const;								//����ģֵƽ��
	Complex Sqrt();											//����ƽ����
	Complex Reciprocal();									//��������
	RtLbsType GetRad() const;								//��ȡ�����ĽǶ�/��λ
	bool IsZero() const;									//�Ƿ�����ֵ

};

inline Complex Exp(RtLbsType y) {
	Complex c;
	c.m_real = cos(y);
	c.m_imag = sin(y);
	return c;
}

#endif
