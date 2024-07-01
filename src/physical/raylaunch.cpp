#include "raylaunch.h"

void RayLaunch_SingleDirection(const uint64_t& rayNum, const Point2D& position, const Vector2D& direction, double theta, std::vector<Ray2D>& outRays)
{
	Vector2D initDir = Rotate(direction, -0.5 * theta);						/** @brief	��ʼ�����߷���	*/
	double rayTheta = theta / rayNum;										/** @brief	���߽Ƕ�	*/
	double rayHalfTheta = 0.5 * rayTheta;									/** @brief	���߰��Ž�	*/
	double rayHalfCosTheta = cos(rayHalfTheta);								/** @brief	���߰��Ž�����	*/
	
	outRays.resize(rayNum);													//��������ȷ��
	for (uint64_t i = 0; i < rayNum; ++i) {
		outRays[i].m_theta = rayHalfTheta;									//��ֵ���߰��Ž�
		outRays[i].m_costheta = rayHalfCosTheta;							//��ֵ���߰��Ž�����
		outRays[i].m_Ori = position;										//��ֵ���߻���λ��
		if (i == 0 || i == rayNum - 1) {									//��ʼ��ת+ĩβ��ת ��תtheta/2
			outRays[i].m_Dir = initDir.Rotate(rayHalfTheta);
		}
		else {																//������ת ��תtheta
			outRays[i].m_Dir = initDir.Rotate(rayTheta);
		}
	}
	return;
}

void RayLaunch_Uniform(const uint64_t& rayNum, const Point2D& position, std::vector<Ray2D>& outRays)
{
	Vector2D initDir(1.0, 0.0, true);										/** @brief	��ʼ���߽Ƕ�	*/
	double rayTheta = TWO_PI / rayNum;										/** @brief	�����Ž�	*/
	double rayHalfTheta = 0.5 * rayTheta;									/** @brief	���߰��Ž�	*/
	double rayHalfCosTheta = cos(rayHalfTheta);								/** @brief	���߰��Ž�����	*/

	outRays.resize(rayNum);													//��������ȷ��
	for (uint64_t i = 0; i < rayNum; ++i) {
		outRays[i].m_theta = rayHalfTheta;
		outRays[i].m_costheta = rayHalfCosTheta;
		outRays[i].m_Ori = position;
		if (i == 0) {
			outRays[i].m_Dir = initDir;
		}
		else {																// ��תtheta
			outRays[i].m_Dir = initDir.Rotate(rayTheta);
		}
	}
	return;
}

void RayLaunch_MultiDirection(uint64_t& rayNum, const Point2D& position, const AngularSpectrum& spectrum, std::vector<Ray2D>& outRays)
{
	//���㵥λ�Ƕ��Ϸ�������������
	int unitRayNum = static_cast<int>(std::ceil(rayNum / spectrum.m_size));
	RtLbsType angleUnit = spectrum.CalUnitAngle();
	RtLbsType rayTheta = angleUnit / unitRayNum;										/** @brief	��λ�Ƕ��ϳ���������Ž�	*/
	RtLbsType rayCosTheta = cos(rayTheta);
	RtLbsType rayHalfTheta = rayTheta / 2.0;											/** @brief	��λ�Ƕ��ϳ�������߰��Ž�	*/
	RtLbsType rayCosHalfTheta = cos(rayHalfTheta);
	uint64_t rayId = 0;
	outRays.resize(unitRayNum * spectrum.m_units.size());
	for (auto it = spectrum.m_units.begin(); it != spectrum.m_units.end(); ++it) {
		const AngularElement& element = *it;
		if (!element.m_isValid) {
			continue;
		}
		Vector2D initDir(element.m_startTheta, true);

		//��תhalftheta
		outRays[rayId].m_Dir = initDir.Rotate(rayHalfTheta);
		outRays[rayId].m_Ori = position;
		outRays[rayId].m_theta = rayHalfTheta;
		outRays[rayId++].m_costheta = rayCosHalfTheta;
		//��תtheta
		for (int i = 1; i < unitRayNum - 1; ++i) {
			outRays[rayId].m_Dir = initDir.Rotate(rayTheta);
			outRays[rayId].m_Ori = position;
			outRays[rayId].m_theta = rayHalfTheta;
			outRays[rayId++].m_costheta = rayCosHalfTheta;
		}
		//��תhalftheta
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
	const Point2D& sPosition = sensor->GetPosition2D();										/** @brief	����������	*/
	//���ݶ�λģʽ�Ĳ�ͬ�����в�ͬ�ķ������
	if (localizeMode == LBS_METHOD_RT_AOA || localizeMode == LBS_METHOD_RT_AOA_TDOA) {		//��������ǶȵĶ�λ���ɰ��սǶȽ����ض�����
		int singleDirectionRayNum = 1;														/** @brief	ÿ���������������	*/
		double thetaofSingleDirection = sensor->m_phiErrorSTD;								/** @brief	ÿ��������Ž�	*/
		if (sensor->m_phiErrorSTD != 0) {
			singleDirectionRayNum = static_cast<int>(std::ceil(sensor->m_phiErrorSTD / rayLaunchTheta));
		}
		int rayNum = singleDirectionRayNum * sDataSize;
		outRays.resize(rayNum);
		double rayHalfTheta = rayLaunchTheta / 2.0;											/** @brief	���߰��Ž�	*/

		for (int i = 0; i < sDataSize; ++i) {
			const SensorData& curSensorData = sensor->m_sensorDataCollection.m_data[i];
			Vector2D initRayDir = curSensorData.GetDirection();
			initRayDir.Rotate(-1 * thetaofSingleDirection / 2.0 - rayHalfTheta);
			for (int j = 0; j < singleDirectionRayNum; ++j) {
				int offset = i * singleDirectionRayNum + j;
				initRayDir.Rotate(rayHalfTheta);
				outRays[offset].m_Ori = sPosition;
				outRays[offset].m_Dir = initRayDir;
				outRays[offset].m_theta = rayHalfTheta;									//���߹ܽǶ�Ϊ�������ĽǶȲ������
				outRays[offset].m_costheta = cos(rayHalfTheta);
				outRays[offset].m_sensorDataId = curSensorData.m_id;								//���д���������ID�ĸ�ֵ
			}
		}
	}
	else if (localizeMode == LBS_METHOD_RT_TDOA) {										//δ��������ǶȵĶ�λ�����վ��ȽǶȽ��з���
		outRays.resize(rayNum);															//��������ȷ��
		Vector2D initDir(1.0, 0.0, true);												/** @brief	��ʼ���߽Ƕ�	*/
		double rayTheta = TWO_PI / rayNum;												/** @brief	�����Ž�	*/
		double rayHalfTheta = 0.5 * rayTheta;											/** @brief	���߰��Ž�	*/
		double rayHalfCosTheta = cos(rayHalfTheta);										/** @brief	���߰��Ž�����	*/
		for (uint64_t i = 0; i < rayNum; ++i) {
			outRays[i].m_theta = rayHalfTheta;
			outRays[i].m_costheta = rayHalfCosTheta;
			outRays[i].m_Ori = sPosition;
			if (i == 0) {
				outRays[i].m_Dir = initDir;
			}
			else {																		// ��תtheta
				outRays[i].m_Dir = initDir.Rotate(rayTheta);
			}
			outRays[i].m_sensorDataId = -2;												//TDOA ����ʱ�趨����Ϊ-2������������Ҫ������չ����ÿ������Դ��Ҫ��չ���е�����
		}
		return;
	}
}
