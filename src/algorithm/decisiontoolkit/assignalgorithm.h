#ifndef RTLBS_ASSIGNALGORITHM
#define RTLBS_ASSIGNALGORITHM

#include "rtlbs.h"
#include "utility/define.h"
#include <glpk.h>										//增加此项由于本程序需要解决指派中的最小代价问题

//采用glpk库进行指派优化,最小代价约束
std::vector<std::pair<int, int>> AssignOprimization(const std::vector<std::vector<RtLbsType>>& cost);

#endif
