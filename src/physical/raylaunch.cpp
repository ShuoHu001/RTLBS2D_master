#include "raylaunch.h"

void RayLaunch_SingleDirection(const uint64_t& rayNum, const Point2D& position, const Vector2D& direction, double theta, std::vector<Ray2D>& outRays)
{
	Vector2D initDir = Rotate(direction, -0.5 * theta);						/** @brief	初始的射线方向	*/
	double rayTheta = theta / rayNum;										/** @brief	射线角度	*/
	double rayHalfTheta = 0.5 * rayTheta;									/** @brief	射线半张角	*/
	double rayHalfCosTheta = cos(rayHalfTheta);								/** @brief	射线半张角余弦	*/
	
	outRays.resize(rayNum);													//射线数量确定
	for (uint64_t i = 0; i < rayNum; ++i) {
		outRays[i].m_theta = rayHalfTheta;									//赋值射线半张角
		outRays[i].m_costheta = rayHalfCosTheta;							//赋值射线半张角余弦
		outRays[i].m_Ori = position;										//赋值射线基点位置
		if (i == 0 || i == rayNum - 1) {									//初始旋转+末尾旋转 旋转theta/2
			outRays[i].m_Dir = initDir.Rotate(rayHalfTheta);
		}
		else {																//常规旋转 旋转theta
			outRays[i].m_Dir = initDir.Rotate(rayTheta);
		}
	}
	return;
}

void RayLaunch_Uniform(const uint64_t& rayNum, const Point2D& position, std::vector<Ray2D>& outRays)
{
	Vector2D initDir(1.0, 0.0, true);										/** @brief	初始射线角度	*/
	double rayTheta = TWO_PI / rayNum;										/** @brief	射线张角	*/
	double rayHalfTheta = 0.5 * rayTheta;									/** @brief	射线半张角	*/
	double rayHalfCosTheta = cos(rayHalfTheta);								/** @brief	射线半张角余弦	*/

	outRays.resize(rayNum);													//射线数量确定
	for (uint64_t i = 0; i < rayNum; ++i) {
		outRays[i].m_theta = rayHalfTheta;
		outRays[i].m_costheta = rayHalfCosTheta;
		outRays[i].m_Ori = position;
		if (i == 0) {
			outRays[i].m_Dir = initDir;
		}
		else {																// 旋转theta
			outRays[i].m_Dir = initDir.Rotate(rayTheta);
		}
	}
	return;
}

void RayLaunch_MultiDirection(uint64_t& rayNum, const Point2D& position, const AngularSpectrum& spectrum, std::vector<Ray2D>& outRays)
{
	//计算单位角度上发出的射线数量
	int unitRayNum = static_cast<int>(std::ceil(rayNum / spectrum.m_size));
	RtLbsType angleUnit = spectrum.CalUnitAngle();
	RtLbsType rayTheta = angleUnit / unitRayNum;										/** @brief	单位角度上出射的射线张角	*/
	RtLbsType rayCosTheta = cos(rayTheta);
	RtLbsType rayHalfTheta = rayTheta / 2.0;											/** @brief	单位角度上出射的射线半张角	*/
	RtLbsType rayCosHalfTheta = cos(rayHalfTheta);
	uint64_t rayId = 0;
	outRays.resize(unitRayNum * spectrum.m_units.size());
	for (auto it = spectrum.m_units.begin(); it != spectrum.m_units.end(); ++it) {
		const AngularElement& element = *it;
		if (!element.m_isValid) {
			continue;
		}
		Vector2D initDir(element.m_startTheta, true);

		//旋转halftheta
		outRays[rayId].m_Dir = initDir.Rotate(rayHalfTheta);
		outRays[rayId].m_Ori = position;
		outRays[rayId].m_theta = rayHalfTheta;
		outRays[rayId++].m_costheta = rayCosHalfTheta;
		//旋转theta
		for (int i = 1; i < unitRayNum - 1; ++i) {
			outRays[rayId].m_Dir = initDir.Rotate(rayTheta);
			outRays[rayId].m_Ori = position;
			outRays[rayId].m_theta = rayHalfTheta;
			outRays[rayId++].m_costheta = rayCosHalfTheta;
		}
		//旋转halftheta
		outRays[rayId].m_Dir = initDir.Rotate(rayHalfTheta);
		outRays[rayId].m_Ori = position;
		outRays[rayId].m_theta = rayHalfTheta;
		outRays[rayId++].m_costheta = rayCosHalfTheta;
	}
	outRays.resize(rayId);
	rayNum = rayId;
}

void RayLaunch_BySensor(LOCALIZATION_METHOD localizeMode, uint64_t rayNum, const Sensor* sensor, RtLbsType rayLaunchTheta, std::vector<Ray2D>& outRays)
{
	int sDataSize = static_cast<int>(sensor->m_sensorDataCollection.m_data.size());
	const Point2D& sPosition = sensor->GetPosition2D();										/** @brief	传感器坐标	*/
	//根据定位模式的不同，进行不同的发射操作
	if (localizeMode == LBS_METHOD_RT_AOA || localizeMode == LBS_METHOD_RT_AOA_TDOA) {		//包含测向角度的定位，可按照角度进行特定发射
		int singleDirectionRayNum = 1;														/** @brief	每个方向的射线数量	*/
		double thetaofSingleDirection = sensor->m_phiErrorSTD;								/** @brief	每个方向的张角	*/
		if (sensor->m_phiErrorSTD != 0) {
			singleDirectionRayNum = static_cast<int>(std::ceil(sensor->m_phiErrorSTD / rayLaunchTheta));
		}
		int rayNum = singleDirectionRayNum * sDataSize;
		outRays.resize(rayNum);
		double rayHalfTheta = rayLaunchTheta / 2.0;											/** @brief	射线半张角	*/

		for (int i = 0; i < sDataSize; ++i) {
			const SensorData& curSensorData = sensor->m_sensorDataCollection.m_data[i];
			Vector2D initRayDir = curSensorData.GetDirection();
			initRayDir.Rotate(-1 * thetaofSingleDirection / 2.0 - rayHalfTheta);
			for (int j = 0; j < singleDirectionRayNum; ++j) {
				int offset = i * singleDirectionRayNum + j;
				initRayDir.Rotate(rayHalfTheta);
				outRays[offset].m_Ori = sPosition;
				outRays[offset].m_Dir = initRayDir;
				outRays[offset].m_theta = rayHalfTheta;									//射线管角度为传感器的角度测量误差
				outRays[offset].m_costheta = cos(rayHalfTheta);
				outRays[offset].m_sensorDataId = curSensorData.m_id;								//进行传感器数据ID的赋值
			}
		}
	}
	else if (localizeMode == LBS_METHOD_RT_TDOA) {										//未包含测向角度的定位，按照均匀角度进行反射
		outRays.resize(rayNum);															//射线数量确定
		Vector2D initDir(1.0, 0.0, true);												/** @brief	初始射线角度	*/
		double rayTheta = TWO_PI / rayNum;												/** @brief	射线张角	*/
		double rayHalfTheta = 0.5 * rayTheta;											/** @brief	射线半张角	*/
		double rayHalfCosTheta = cos(rayHalfTheta);										/** @brief	射线半张角余弦	*/
		for (uint64_t i = 0; i < rayNum; ++i) {
			outRays[i].m_theta = rayHalfTheta;
			outRays[i].m_costheta = rayHalfCosTheta;
			outRays[i].m_Ori = sPosition;
			if (i == 0) {
				outRays[i].m_Dir = initDir;
			}
			else {																		// 旋转theta
				outRays[i].m_Dir = initDir.Rotate(rayTheta);
			}
			outRays[i].m_sensorDataId = -2;												//TDOA 方法时设定数据为-2，代表数据需要进行扩展，即每个广义源都要扩展所有的数据
		}
		return;
	}
}
