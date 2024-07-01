#ifndef RTLBS_RAYLAUNCH
#define RTLBS_RAYLAUNCH

//��ͷ�ļ����ڶ������߷�����������漰�ĺ���

#include "rtlbs.h"
#include "utility/define.h"
#include "utility/enum.h"
#include "geometry/point2d.h"
#include "geometry/vector2d.h"
#include "geometry/ray2d.h"
#include "angularspectrum/angularspectrum.h"
#include "equipment/sensor.h"



void RayLaunch_SingleDirection(const uint64_t& rayNum, const Point2D& position, const Vector2D& direction, double theta, std::vector<Ray2D>& outRays);

void RayLaunch_Uniform(const uint64_t& rayNum, const Point2D& position, std::vector<Ray2D>& outRays);

void RayLaunch_MultiDirection(uint64_t& rayNum, const Point2D& position, const AngularSpectrum& spectrum, std::vector<Ray2D>& outRays);

//���ڴ��������з�������
void RayLaunch_BySensor(LOCALIZATION_METHOD localizeMode, uint64_t rayNum, const Sensor* sensor, RtLbsType rayLaunchTheta, std::vector<Ray2D>& outRays);
#endif
