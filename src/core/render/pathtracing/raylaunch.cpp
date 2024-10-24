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

void RayLaunch_BySensor(LOCALIZATION_METHOD lbsMethod, LOCALIZATION_MODE lbsMode, uint64_t rayNum, const Sensor* sensor, RtLbsType rayLaunchTheta, std::vector<Ray2D>& outRays)
{
	int sDataSize = static_cast<int>(sensor->m_sensorDataCollection.m_datas.size());
	const Point2D& sPosition = sensor->GetPosition2D();										/** @brief	����������	*/
	//���ݶ�λģʽ�Ĳ�ͬ�����в�ͬ�ķ������
	if (lbsMethod == LBS_METHOD_RT_AOA || lbsMethod == LBS_METHOD_RT_AOA_TDOA) {		//��������ǶȵĶ�λ���ɰ��սǶȽ����ض�����
		int singleDirectionRayNum = 1;														/** @brief	ÿ���������������,Ĭ��Ϊ1	*/
		double thetaofSingleDirection = sensor->m_phiErrorSTD;								/** @brief	ÿ��������Ž�	*/
		if (sensor->m_phiErrorSTD != 0) {
			singleDirectionRayNum = static_cast<int>(std::ceil(sensor->m_phiErrorSTD / rayLaunchTheta));
		}
		int rayNum = singleDirectionRayNum * sDataSize;
		outRays.resize(rayNum);
		double rayHalfTheta = rayLaunchTheta / 2.0;											/** @brief	���߰��Ž�	*/

		for (int i = 0; i < sDataSize; ++i) {
			const SensorData& curSensorData = sensor->m_sensorDataCollection.m_datas[i];
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
	else if (lbsMethod == LBS_METHOD_RT_TOA) {
		if (lbsMode == LBS_MODE_MPSTSD) {				//�㷨ΪTOA�㷨ʱ��ÿ��վ��������ͬ
			outRays.resize(rayNum);															//��������ȷ��
			Vector2D initDir(1.0, 0.0, true);												/** @brief	��ʼ���߽Ƕ�	*/
			double rayTheta = TWO_PI / rayNum;												/** @brief	�����Ž�	*/
			double rayHalfTheta = 0.5 * rayTheta;											/** @brief	���߰��Ž�	*/
			double rayHalfCosTheta = cos(rayHalfTheta);										/** @brief	���߰��Ž�����	*/
			const SensorData& curSensorData = sensor->m_sensorDataCollection.m_datas[0];
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
				outRays[i].m_tLimit = curSensorData.m_time * LIGHT_VELOCITY_AIR;
				outRays[i].m_sensorDataId = curSensorData.m_id;								//���д���������ID�ĸ�ֵ
			}
		}
		else if (lbsMode == LBS_MODE_SPSTMD) {			//�㷨ΪTOA�㷨ʱ����վͬʱӵ�ж������
			outRays.resize(rayNum);															//��������ȷ��
			Vector2D initDir(1.0, 0.0, true);												/** @brief	��ʼ���߽Ƕ�	*/
			double rayTheta = TWO_PI / rayNum;												/** @brief	�����Ž�	*/
			double rayHalfTheta = 0.5 * rayTheta;											/** @brief	���߰��Ž�	*/
			double rayHalfCosTheta = cos(rayHalfTheta);										/** @brief	���߰��Ž�����	*/
			RtLbsType max_propagation_time = sensor->m_sensorDataCollection.GetMaxPropagationTime();
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
				outRays[i].m_tLimit = max_propagation_time * LIGHT_VELOCITY_AIR;
				outRays[i].m_sensorDataId = -2;												//�趨����Ϊ-2������������Ҫ������չ����ÿ������Դ��Ҫ��չ���е�����
			}
		}
		return;
	}
	else if (lbsMethod == LBS_METHOD_RT_AOA_TOA) {
		int singleDirectionRayNum = 1;														/** @brief	ÿ���������������,Ĭ��Ϊ1	*/
		double thetaofSingleDirection = sensor->m_phiErrorSTD;								/** @brief	ÿ��������Ž�	*/
		if (sensor->m_phiErrorSTD != 0) {
			singleDirectionRayNum = static_cast<int>(std::ceil(sensor->m_phiErrorSTD / rayLaunchTheta));
		}
		int rayNum = singleDirectionRayNum * sDataSize;
		outRays.resize(rayNum);
		double rayHalfTheta = rayLaunchTheta / 2.0;											/** @brief	���߰��Ž�	*/

		for (int i = 0; i < sDataSize; ++i) {
			const SensorData& curSensorData = sensor->m_sensorDataCollection.m_datas[i];
			Vector2D initRayDir = curSensorData.GetDirection();
			initRayDir.Rotate(-1 * thetaofSingleDirection / 2.0 - rayHalfTheta);
			for (int j = 0; j < singleDirectionRayNum; ++j) {
				int offset = i * singleDirectionRayNum + j;
				initRayDir.Rotate(rayHalfTheta);
				outRays[offset].m_Ori = sPosition;
				outRays[offset].m_Dir = initRayDir;
				outRays[offset].m_theta = rayHalfTheta;									//���߹ܽǶ�Ϊ�������ĽǶȲ������
				outRays[offset].m_costheta = cos(rayHalfTheta);
				outRays[offset].m_tLimit = curSensorData.m_time * LIGHT_VELOCITY_AIR;				//�������ߴ������룬ʵ����Ҳ��TOA��ֵ
				outRays[offset].m_sensorDataId = curSensorData.m_id;								//���д���������ID�ĸ�ֵ
			}
		}
	}
	else if (lbsMethod == LBS_METHOD_RT_TDOA) {										   
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
