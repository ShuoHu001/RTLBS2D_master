#ifndef RTLBS_ASSIGNALGORITHM
#define RTLBS_ASSIGNALGORITHM

#include "rtlbs.h"
#include "utility/define.h"
#include <glpk.h>										//���Ӵ������ڱ�������Ҫ���ָ���е���С��������

//����glpk�����ָ���Ż�,��С����Լ��
std::vector<std::pair<int, int>> AssignOprimization(const std::vector<std::vector<RtLbsType>>& cost);

#endif
