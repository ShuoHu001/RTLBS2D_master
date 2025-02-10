#include "testlbs.h"

void TestAOALocalizationSingleStationInDifferentError()
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	std::vector<RtLbsType> phiDegreeErrors = { 0.1,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0 };
	std::vector<RtLbsType> powerErrors = { 1.0 };
	//std::vector<RtLbsType> phiDegreeErrors = { 4.0 };
	//std::vector<RtLbsType> powerErrors = { 0 };

	std::string positionName = "A";
	int roundTime = 400;
	Point2D realPoint = { 76,56 };

	int totalround = static_cast<int>(phiDegreeErrors.size() * powerErrors.size() * roundTime);

	int roundi = 0;
	for (auto phiDegreeError : phiDegreeErrors) {
		for (auto powerError : powerErrors) {
			RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
			std::vector<LBSData> datas;
			datas.reserve(roundTime);

			//��ȡ��������Ϣ��д���Ӧ���������
			SensorCollectionConfig curSensorCollectionConfig;
			curSensorCollectionConfig.Init("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
			curSensorCollectionConfig.m_sensorConfigs[0].m_phiDegreeErrorSTD = phiDegreeError;
			curSensorCollectionConfig.m_sensorConfigs[0].m_powerErrorSTD = powerError+2;
			curSensorCollectionConfig.Write2Json("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
			System* system;
			for (int i = 0; i < roundTime; ++i) {
				std::cout << roundi++ / static_cast<RtLbsType>(totalround) * 100 << std::endl;			//�㱨����

				LBSData curData;
				curData.truePosition = realPoint;

				system = new System();
				if (!system->Setup(MODE_LBS))
					return;
				system->Render();
				Point2D curTargetPoint = system->TargetLocalization(LBS_MODE_SPSTMD, LBS_METHOD_RT_AOA);
				if (!system->m_scene->IsValidPoint(curTargetPoint)) {
					delete system;
					continue;
				}
				RtLbsType error = (realPoint - curTargetPoint).Length();
				if (error < 200) {
					curData.targetPosition = curTargetPoint;
					curData.error = error;
					datas.push_back(curData);
				}
				delete system;
			}

			//����ƽ�����
			RtLbsType errorMean = 0.0;
			for (auto& data : datas) {
				errorMean += data.error;
			}
			errorMean /= roundTime;

			//д�������ļ�
			std::stringstream ss;
			ss << "��λ���ܷ���/����������/" << positionName << "_phiError_" << phiDegreeError << "_powerError_" << powerError << ".txt";
			std::ofstream outFile(ss.str());
			for (auto& data : datas) {
				data.Write2File(outFile);
			}
			outFile.close();
			LOG_INFO << errorMean << std::endl;
		}
	}
	LOG_INFO << "�������" << ENDL;
}

void TestAOAlocalizationMultiStationInDifferentError()
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{37,30} };
	std::vector<std::string> postionNames = { "A","B","C" };
	int positionId = 2;

	std::vector<RtLbsType> phiDegreeErrors = { /*0.1,1.0,2.0,3.0,4.0,5.0,6.0,7.0,*/8.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	int roundTime = 100;
	System* lbsSystem = nullptr;
	for (auto phiDegreeError : phiDegreeErrors) {
		for (auto powerError : powerErrors) {
			RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
			std::vector<LBSData> datas;
			datas.resize(roundTime);
			//��ȡ��������Ϣ��д���Ӧ���������
			for (int i = 0; i < roundTime; ++i) {
				datas[i].truePosition = realPositions[positionId];
				SensorCollectionConfig curSensorConfig;
				curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
				for (int j = 0; j < stationNum; ++j) {
					curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeError;
					curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiError;
					curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = 0.2;
					curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
					curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_aoa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
				}

				curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
				lbsSystem = new System();
				lbsSystem->Setup(MODE_LBS);
				lbsSystem->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = phiDegreeError;
				lbsSystem->Render();
				Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA);
				datas[i].targetPosition = targetPosition;
				datas[i].error = (targetPosition - datas[i].truePosition).Length();
				if (datas[i].error > 200) {
					datas[i].isValid = false;
				}
				std::cout << i << std::endl;
				delete lbsSystem;
			}
			std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(phiDegreeError) + "_powerError_" + std::to_string(powerError) + ".txt";
			std::ofstream outstream(outputFileName);
			for (auto& data : datas) {
				if (data.isValid) {
					data.Write2File(outstream);
				}
			}
			outstream.close();
		}
	}

}

void TestTOALocalizationSingleStationInDifferentError(int positionId)
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	std::vector<Point2D> realPoints = { {76,56},{121,74},{70,90} };
	std::vector<std::string> postionNames = { "A","B","C" };

	//Ԥд����
	//��ȡ�������ã�д���Ӧ����ⷽ��
	SimConfig simconfig;
	simconfig.Init("config/sysconfig.json");
	simconfig.m_systemMode = MODE_LBS;
	simconfig.m_lbsConfig.m_lbsMode = LBS_MODE_SPSTMD;
	simconfig.m_lbsConfig.m_lbsMethod = LBS_METHOD_RT_TOA;
	simconfig.m_lbsConfig.m_threadNum = 20;
	simconfig.Writer2Json("config/sysconfig.json");

	std::vector<RtLbsType> timeNSErrors = { 2,4,6,8,10,12,14,16,18,20 };
	std::vector<RtLbsType> powerErrors = { 6 };

	std::string positionName = postionNames[positionId];
	int roundTime = 400;

	Point2D realPoint = realPoints[positionId];

	int totalround = static_cast<int>(timeNSErrors.size() * powerErrors.size() * roundTime);

	int roundi = 0;
	for (auto timeError : timeNSErrors) {
		for (auto powerError : powerErrors) {
			std::vector<LBSData> datas;
			datas.reserve(roundTime);

			//��ȡ��������Ϣ��д���Ӧ���������
			SensorCollectionConfig curSensorCollectionConfig;
			curSensorCollectionConfig.Init("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
			curSensorCollectionConfig.m_sensorConfigs[0].m_timeErrorSTD = timeError;
			curSensorCollectionConfig.m_sensorConfigs[0].m_powerErrorSTD = powerError;
			curSensorCollectionConfig.m_sensorConfigs[0].m_sensorDataFileName = "results/rt/sensor data/SPSTMD/tx" + std::to_string(positionId) + "_SPSTMD.json";
			curSensorCollectionConfig.Write2Json("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
			System* system;
			for (int i = 0; i < roundTime; ++i) {
				std::cout << roundi++ / static_cast<RtLbsType>(totalround) * 100 << std::endl;			//�㱨����

				LBSData curData;
				curData.truePosition = realPoint;

				system = new System();
				if (!system->Setup(MODE_LBS))
					return;
				system->Render();
				Point2D curTargetPoint = system->TargetLocalization(LBS_MODE_SPSTMD, LBS_METHOD_RT_TOA);
				RtLbsType error = (realPoint - curTargetPoint).Length();
				curData.targetPosition = curTargetPoint;
				curData.error = error;
				datas.push_back(curData);
				delete system;
			}

			//����ƽ�����
			RtLbsType errorMean = 0.0;
			for (auto& data : datas) {
				errorMean += data.error;
			}
			errorMean /= roundTime;

			//д�������ļ�
			std::stringstream ss;
			ss << "��λ���ܷ���/����������/" << positionName << "_timeError_" << timeError << "_powerError_" << powerError << ".txt";
			std::ofstream outFile(ss.str());
			for (auto& data : datas) {
				data.Write2File(outFile);
			}
			outFile.close();
			LOG_INFO << errorMean << std::endl;
		}
	}
	LOG_INFO << "�������" << ENDL;
}

void TestTOALocalizationMultiStationInDifferentError()
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{38,28} };
	std::vector<std::string> postionNames = { "A","B","C" };
	int positionId = 0;

	std::vector<RtLbsType> phiDegreeErrors = { 0.1,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	std::vector<RtLbsType> timeErrors = { 3.0 };
	int roundTime = 20;
	System* lbsSystem = nullptr;
	for (auto phiDegreeError : phiDegreeErrors) {
		for (auto timeError : timeErrors) {
			for (auto powerError : powerErrors) {
				RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
				std::vector<LBSData> datas;
				datas.resize(roundTime);
				//��ȡ��������Ϣ��д���Ӧ���������
				for (int i = 0; i < roundTime; ++i) {
					datas[i].truePosition = realPositions[positionId];
					SensorCollectionConfig curSensorConfig;
					curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
					for (int j = 0; j < stationNum; ++j) {
						curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeError;
						curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiError;
						curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = timeError;
						curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
						curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_toa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
					}
					curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
					lbsSystem = new System();
					lbsSystem->Setup(MODE_LBS);
					lbsSystem->Render();
					Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_TOA);
					datas[i].targetPosition = targetPosition;
					datas[i].error = (targetPosition - datas[i].truePosition).Length();
					if (datas[i].error > 5) {
						datas[i].isValid = false;
					}
					std::cout << i << std::endl;
					delete lbsSystem;
				}
				std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(phiDegreeError) + "_timeError_" + std::to_string(timeError) + "_powerError_" + std::to_string(powerError) + ".txt";
				std::ofstream outstream(outputFileName);
				for (auto& data : datas) {
					if (data.isValid) {
						data.Write2File(outstream);
					}
				}
				outstream.close();
			}
		}
	}
}

void TestAOATDOALocalizationSingleStationInDifferentError()
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	std::vector<RtLbsType> timeNSErrors = { 3 };
	std::vector<RtLbsType> phiDegreeErrors = { 0.1, 1, 2,3,4,5,6,7,8 };
	std::vector<RtLbsType> powerErrors = { 3 };
	//std::vector<RtLbsType> phiDegreeErrors = { 4.0 };
	//std::vector<RtLbsType> powerErrors = { 0 };

	std::string positionName = "A";
	int roundTime = 400;
	Point2D realPoint = { 76,56 };

	int totalround = static_cast<int>(phiDegreeErrors.size() * timeNSErrors.size() * powerErrors.size() * roundTime);

	int roundi = 0;
	for (auto timeError : timeNSErrors) {
		for (auto powerError : powerErrors) {
			for (auto phiDegreeError : phiDegreeErrors) {
				std::vector<LBSData> datas;
				datas.reserve(roundTime);

				//��ȡ��������Ϣ��д���Ӧ���������
				SensorCollectionConfig curSensorCollectionConfig;
				curSensorCollectionConfig.Init("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
				curSensorCollectionConfig.m_sensorConfigs[0].m_phiDegreeErrorSTD = phiDegreeError;
				curSensorCollectionConfig.m_sensorConfigs[0].m_timeErrorSTD = timeError;
				curSensorCollectionConfig.m_sensorConfigs[0].m_powerErrorSTD = powerError;
				curSensorCollectionConfig.Write2Json("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
				System* system;
				int validRoundTime = 0;
				for (int i = 0; i < roundTime; ++i) {
					std::cout << roundi++ / static_cast<RtLbsType>(totalround) * 100 << std::endl;			//�㱨����
					LBSData curData;
					curData.truePosition = realPoint;

					system = new System();
					if (!system->Setup(MODE_LBS))
						return;
					system->Render();
					Point2D curTargetPoint = system->TargetLocalization(LBS_MODE_SPSTMD, LBS_METHOD_RT_AOA_TDOA);
					RtLbsType error = (realPoint - curTargetPoint).Length();
					if (error < 200) {
						curData.targetPosition = curTargetPoint;
						curData.error = error;
						datas.push_back(curData);
						validRoundTime++;
					}
					delete system;
				}

				//����ƽ�����
				RtLbsType errorMean = 0.0;
				for (auto& data : datas) {
					errorMean += data.error;
				}
				errorMean /= validRoundTime;

				//д�������ļ�
				std::stringstream ss;
				ss << "��λ���ܷ���/����������/" << positionName<<"_phiError_" << phiDegreeError << "_timeError_" << timeError << "_powerError_" << powerError << ".txt";
				std::ofstream outFile(ss.str());
				for (auto& data : datas) {
					data.Write2File(outFile);
				}
				outFile.close();
				LOG_INFO << errorMean << std::endl;
			}
		}
	}
	LOG_INFO << "�������" << ENDL;
}

void TestAOATDOALocalizationMultiStationInDifferentError()
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{37,30} };
	std::vector<std::string> postionNames = { "A","B","C" };
	int positionId = 1;

	std::vector<RtLbsType> phiDegreeErrors = { 0.1,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	std::vector<RtLbsType> timeErrors = { 3.0 };

	int roundTime = 100;
	System* lbsSystem = nullptr;
	for (auto phiDegreeError : phiDegreeErrors) {
		for (auto timeError : timeErrors) {
			for (auto powerError : powerErrors) {
				RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
				std::vector<LBSData> datas;
				datas.resize(roundTime);
				//��ȡ��������Ϣ��д���Ӧ���������
				for (int i = 0; i < roundTime; ++i) {
					datas[i].truePosition = realPositions[positionId];
					SensorCollectionConfig curSensorConfig;
					curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
					for (int j = 0; j < stationNum; ++j) {
						curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeError;
						curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiError;
						curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = timeError;
						curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
						curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_toa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
					}
					curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
					lbsSystem = new System();
					lbsSystem->Setup(MODE_LBS);
					lbsSystem->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = phiDegreeError;
					lbsSystem->Render();
					Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA_TDOA);
					datas[i].targetPosition = targetPosition;
					datas[i].error = (targetPosition - datas[i].truePosition).Length();
					if (datas[i].error > 2) {
						datas[i].isValid = false;
					}
					std::cout << i << std::endl;
					delete lbsSystem;
				}
				std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(phiDegreeError) + "_timeError_" + std::to_string(timeError) + "_powerError_" + std::to_string(powerError) + ".txt";
				std::ofstream outstream(outputFileName);
				for (auto& data : datas) {
					if (data.isValid) {
						data.Write2File(outstream);
					}
				}
				outstream.close();
			}
		}
	}
}

void TestAOATOALocalizationSingleStationInDifferentError()
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	std::vector<RtLbsType> timeNSErrors = { 3 };
	std::vector<RtLbsType> phiDegreeErrors = { /*0.1, 1, */2,3,4,5,6,7,8 };
	//std::vector<RtLbsType> phiDegreeErrors = { 1.0 };
	std::vector<RtLbsType> powerErrors = { 3 };
	//std::vector<RtLbsType> phiDegreeErrors = { 4.0 };
	//std::vector<RtLbsType> powerErrors = { 0 };

	std::string positionName = "A";
	int roundTime = 400;
	Point2D realPoint = { 76,56 };

	int totalround = static_cast<int>(phiDegreeErrors.size() * timeNSErrors.size() * powerErrors.size() * roundTime);

	int roundi = 0;
	for (auto timeError : timeNSErrors) {
		for (auto powerError : powerErrors) {
			for (auto phiDegreeError : phiDegreeErrors) {
				std::vector<LBSData> datas;
				datas.reserve(roundTime);

				//��ȡ��������Ϣ��д���Ӧ���������
				SensorCollectionConfig curSensorCollectionConfig;
				curSensorCollectionConfig.Init("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
				curSensorCollectionConfig.m_sensorConfigs[0].m_phiDegreeErrorSTD = phiDegreeError;
				curSensorCollectionConfig.m_sensorConfigs[0].m_timeErrorSTD = timeError+4;
				curSensorCollectionConfig.m_sensorConfigs[0].m_powerErrorSTD = powerError;
				curSensorCollectionConfig.Write2Json("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
				System* system;
				int validRoundTime = 0;
				for (int i = 0; i < roundTime; ++i) {
					std::cout << roundi++ / static_cast<RtLbsType>(totalround) * 100 << std::endl;			//�㱨����

					LBSData curData;
					curData.truePosition = realPoint;

					system = new System();
					if (!system->Setup(MODE_LBS))
						return;
					system->Render();
					Point2D curTargetPoint = system->TargetLocalization(LBS_MODE_SPSTMD, LBS_METHOD_RT_AOA_TOA);
					RtLbsType error = (realPoint - curTargetPoint).Length();
					if (error < 200) {
						curData.targetPosition = curTargetPoint;
						curData.error = error;
						datas.push_back(curData);
						validRoundTime++;
					}
					delete system;
				}

				//����ƽ�����
				RtLbsType errorMean = 0.0;
				for (auto& data : datas) {
					errorMean += data.error;
				}
				errorMean /= validRoundTime;

				//д�������ļ�
				std::stringstream ss;
				ss << "��λ���ܷ���/����������/" << positionName << "_phiError_" << phiDegreeError << "_timeError_" << timeError << "_powerError_" << powerError << ".txt";
				std::ofstream outFile(ss.str());
				for (auto& data : datas) {
					data.Write2File(outFile);
				}
				outFile.close();
				LOG_INFO << errorMean << std::endl;
			}
		}
	}
	LOG_INFO << "�������" << ENDL;
}

void TestAOATOALocalizationMultiStationInDifferentError()
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{37,30} };
	std::vector<std::string> postionNames = { "A","B","C" };
	int positionId = 2;

	std::vector<RtLbsType> phiDegreeErrors = { /*0.1,1.0,2.0,3.0,*/4.0,5.0,6.0,7.0,8.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	std::vector<RtLbsType> timeErrors = { 3.0 };


	int roundTime = 100;
	System* lbsSystem = nullptr;
	for (auto phiDegreeError : phiDegreeErrors) {
		for (auto timeError : timeErrors) {
			for (auto powerError : powerErrors) {
				RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
				std::vector<LBSData> datas;
				datas.resize(roundTime);
				//��ȡ��������Ϣ��д���Ӧ���������
				for (int i = 0; i < roundTime; ++i) {
					datas[i].truePosition = realPositions[positionId];
					SensorCollectionConfig curSensorConfig;
					curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
					for (int j = 0; j < stationNum; ++j) {
						curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeError;
						curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiError;
						curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = timeError;
						curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
						curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_toa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
					}
					curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
					lbsSystem = new System();
					lbsSystem->Setup(MODE_LBS);
					lbsSystem->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = phiDegreeError;
					lbsSystem->Render();
					Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA_TOA);
					datas[i].targetPosition = targetPosition;
					datas[i].error = (targetPosition - datas[i].truePosition).Length();
					std::cout << i << std::endl;
					delete lbsSystem;
				}
				std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(phiDegreeError) + "_timeError_" + std::to_string(timeError) + "_powerError_" + std::to_string(powerError) + ".txt";
				std::ofstream outstream(outputFileName);
				for (auto& data : datas) {
					if (data.error > 10) {
						continue;
					}
					data.Write2File(outstream);
				}
				outstream.close();
			}
		}
	}
}

void TestAOALocalizaitonSingleStationErrorInDifferentPlace()
{
	struct LBSData{
		bool isValid;
		int sensorDataId;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData():isValid(true),sensorDataId(-1),error(0){}
		LBSData(const Point2D& p) :isValid(true), sensorDataId(-1), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	bool isAlreadyGenerateSensorData = true;				/** @brief	�Ƿ��Ѿ����ɴ���������	*/

	//���ɶ�λ���꼯
	std::vector<LBSData*> datas;
	RtLbsType xmin = 15;
	RtLbsType xmax = 135;
	RtLbsType ymin = 15;
	RtLbsType ymax = 95;
	RtLbsType gap = 1;
	for (RtLbsType y = ymin; y < ymax; y += gap) {
		for (RtLbsType x = xmin; x < xmax; x += gap) {
			LBSData* lbsData = new LBSData(Point2D(x, y));
			datas.push_back(lbsData);
		}
	}
	
	System* rtSystem = new System();
	rtSystem->Setup(MODE_RT);												//��Ҫ��֤����������������
	TransmitterCollectionConfig txCollectionConfig = rtSystem->m_simConfig.m_transmitterConfig;
	TransmitterConfig curTxConfig = txCollectionConfig.m_transmitterConfigs[0];
	txCollectionConfig.m_transmitterConfigs.clear();
	int validSensorDataId = 0;
	for (auto& data : datas) {
		if (!rtSystem->m_scene->IsValidPoint(data->truePosition)) {
			data->isValid = false;
			continue;
		}
		TransmitterConfig newTxConfig = curTxConfig;
		newTxConfig.m_position.x = data->truePosition.x;
		newTxConfig.m_position.y = data->truePosition.y;
		txCollectionConfig.m_transmitterConfigs.push_back(newTxConfig);
		data->sensorDataId = validSensorDataId++;
	}
	delete rtSystem;


	//------------------------------------------��������������----------------------------------------------------------------------------------------
	//�޸�Ϊѭ��ģʽ

	if (!isAlreadyGenerateSensorData) {
		for (int i = 0; i < txCollectionConfig.m_transmitterConfigs.size(); ++i) {
			TransmitterCollectionConfig newTxConfig;
			newTxConfig.m_transmitterConfigs.push_back(txCollectionConfig.m_transmitterConfigs[i]);
			newTxConfig.Write2Json("config/transmitterconfig.json");
			rtSystem = new System();
			rtSystem->Setup(MODE_RT);
			rtSystem->Render();
			rtSystem->PostProcessing();
			rtSystem->OutputResults();
			delete rtSystem;
			//�������ļ�����
			std::string newFileName = "results/rt/sensor data/SPSTMD/tx" + std::to_string(i+1) + "_SPSTMD.json";
			rename("results/rt/sensor data/SPSTMD/tx0_SPSTMD.json", newFileName.c_str());
		}

	}
	//-----------------------------------------------------------------------------------------------------------------------------------------------

	//�޸����ɴ��������ݵ��ļ�
	SimConfig curSimConfig;
	curSimConfig.Init("config/sysconfig.json");
	curSimConfig.SetSeneorConfigFile("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
	curSimConfig.Writer2Json("config/sysconfig.json");

	RtLbsType phiDegreeError = 6.0;
	RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
	RtLbsType powerError = 0.0;
	//ѭ�����㶨λ���
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
	std::string sensorDataFileName;
	int round = 0;
	std::cout << "ok" << std::endl;

	for (auto& data : datas) {
		std::cout << round++ << std::endl;
		
		if (!data->isValid) {
			continue;
		}
		sensorDataFileName = "results/rt/sensor data/SPSTMD/tx" + std::to_string(data->sensorDataId+1) + "_SPSTMD.json";
		curSensorConfig.m_sensorConfigs[0].m_sensorDataFileName = sensorDataFileName;
		curSensorConfig.m_sensorConfigs[0].m_phiDegreeErrorSTD = phiDegreeError;
		curSensorConfig.m_sensorConfigs[0].m_phiErrorSTD = phiError;
		curSensorConfig.m_sensorConfigs[0].m_powerErrorSTD = powerError;
		curSensorConfig.Write2Json("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
		System* lbsSystem = new System();
		
		lbsSystem->Setup(MODE_LBS);
		if (lbsSystem->m_scene->m_sensors[0]->m_sensorDataCollection.m_datas.size() < 2) {						//���ݲ�����ֻ��һ���ྶ������
			data->isValid = false;
			delete lbsSystem;
			continue;
		}
		lbsSystem->Render();
		Point2D targetPosition;
		targetPosition = lbsSystem->TargetLocalization(LBS_MODE_SPSTMD, LBS_METHOD_RT_AOA);
		delete lbsSystem;
		data->targetPosition = targetPosition;
		data->error = (targetPosition - data->truePosition).Length();
	}
	//������д�����ļ���
	std::string outputFileName = "��λ���ܷ���/ȫ��������/phiError_" + std::to_string(phiDegreeError) + "_powerError_" + std::to_string(powerError) + "ԭʼ.txt";
	std::ofstream outstream(outputFileName);
	for (auto& data : datas) {
		data->Write2File(outstream);
	}
	outstream.close();

	std::cout << "�������" << std::endl;
}

void ResearchMultipathSimilarityInLocalizationInDifferentPlaces()
{
	//���ͷ���ģʽ
	System* system = new System();
	system->Setup(MODE_RT);
	system->Render();
	system->PostProcessing();

	int resultSize = system->m_result.m_raytracingResult.size();
	std::vector<ReceiverInfo*> rxInfos(resultSize);

	for (int i = 0; i < resultSize; ++i) {
		rxInfos[i] = new ReceiverInfo(system->m_result.m_raytracingResult[i]);
	}

	//����ྶ���ƶȾ���

	for (int i = 0; i < resultSize; ++i) {
		ReceiverInfo* mainInfo = rxInfos[i];
		if (!mainInfo->m_isValid) {
			continue;
		}
		for (int j = 0; j < resultSize; ++j) {
			if (i == j) {
				continue;
			}
			ReceiverInfo* subInfo = rxInfos[j];
			if (mainInfo->CanAddToSimilarities(rxInfos[j])) {
				mainInfo->m_similarities.push_back(subInfo);
			}
		}
		std::cout << i << std::endl;
	}

	//���¾���
	for (int i = 0; i < resultSize; ++i) {
		ReceiverInfo* curInfo = rxInfos[i];
		curInfo->UpdateSimilaritiesDistance();
	}

	std::ofstream stream("��λ���ܷ���/ȫ�����������/errormatrix.txt");
	for (auto& curInfo : rxInfos) {
		curInfo->Write2File(stream);
	}
	stream.close();

	//�ֱ���㲻ͬ����Ķྶ�������ĽǶ�ƫ�ͳ�Ʋ�ͬ�Ƕ�ƫ���Ӧ��λ�þ���
	std::vector<RtLbsType> phiDegreeErrors = { 0.1,0.2,0.5,1.0,2.0,3.0,4.0,5.0,6.0 };
	for (auto& curPhiError : phiDegreeErrors) {
		std::ofstream newStream("��λ���ܷ���/ȫ�����������/" + std::to_string(curPhiError) + "_errormatrix.txt");
		for (auto& curInfo : rxInfos) {
			if (!curInfo->m_isValid) { 
				newStream << curInfo->m_point.x << "\t" << curInfo->m_point.y << "\t" << 0.0 << std::endl;
				continue;
			}
			RtLbsType curMaxDistance = 0.0;
			RtLbsType curMeanDistance = 0.0;
			curInfo->GetDistanceByPhi(curPhiError * ONE_DEGEREE, curMaxDistance, curMeanDistance);
			newStream << curInfo->m_point.x << "\t" << curInfo->m_point.y << "\t" << curMaxDistance << std::endl;
		}
		newStream.close();
	}
	//���
	std::cout << "finished" << std::endl;
}

void GeneratePowerDataofUWBSystem()
{
	//��ȡ���������
	std::vector<Point3D> points;
	std::ifstream instream("tx_position.txt");
	std::string line;
	while (std::getline(instream, line)) {
		if (line.empty()) continue;    // ��������
		std::istringstream iss(line);  // ʹ���ַ���������ÿһ��
		Point3D point;
		if (iss >> point.x >> point.y >> point.z) { // ��ȡ x, y, z ����
			points.push_back(point);   // ����洢��������
		}
		else {
			std::cerr << "���ݸ�ʽ�����޷�����: " << line << std::endl;
		}
	}
	instream.close();
	int txNum = points.size();

	std::vector<RtLbsType> receivedPower(txNum);
	TransmitterCollectionConfig txCollectionConfig;
	txCollectionConfig.m_transmitterConfigs.resize(1);
	System* system = nullptr;
	for (int i = 0; i < txNum; ++i) {
		txCollectionConfig.m_transmitterConfigs[0].m_position = points[i];
		txCollectionConfig.Write2Json("config/transmitterconfig.json");
		system = new System();
		if (!system->Setup(MODE_RT))
			return;
		system->Render();
		system->PostProcessing();
		system->OutputResults();
		if (!system->m_result.m_raytracingResult[0].m_multipathInfo.empty()) {
			receivedPower[i] = system->m_result.m_raytracingResult[0].m_multipathInfo[0].m_power;
		}
		else {
			receivedPower[i] = -120;
		}
		delete system;
		std::cout << i << std::endl;
	}

	std::ofstream outstream("rx1.txt");
	for (int i = 0; i < txNum; ++i) {
		outstream << i << "\t" << receivedPower[i] << std::endl;
	}
	outstream.close();
}

void GenerateUWBAOALocalizationMultiStationSensorData()
{
	//���ɴ���������

	//��ȡ��������ݣ�����RT�㷨�ӿڣ����ɴ���������
	std::vector<Point3D> points;
	std::ifstream instream("tx_position.txt");
	std::string line;
	while (std::getline(instream, line)) {
		if (line.empty()) continue;    // ��������
		std::istringstream iss(line);  // ʹ���ַ���������ÿһ��
		Point3D point;
		if (iss >> point.x >> point.y >> point.z) { // ��ȡ x, y, z ����
			points.push_back(point);   // ����洢��������
		}
		else {
			std::cerr << "���ݸ�ʽ�����޷�����: " << line << std::endl;
		}
	}
	instream.close();
	int txNum = points.size();
	System* system = nullptr;
	TransmitterCollectionConfig txCollectionConfig;
	txCollectionConfig.m_transmitterConfigs.resize(1);
	std::vector< std::vector<SensorDataCollection>> sensorDatas;
	sensorDatas.resize(txNum);
	for (int i = 0; i < txNum; ++i) {
		txCollectionConfig.m_transmitterConfigs[0].m_position = points[i];
		txCollectionConfig.Write2Json("config/transmitterconfig.json");
		system = new System();
		if (!system->Setup(MODE_RT))
			return;
		system->Render();
		system->PostProcessing();
		sensorDatas[i] = system->m_result.m_sensorDataMPSTSD;
		delete system;
		std::cout << i << std::endl;
	}


	for (int i = 0; i < txNum; ++i) {
		for (int j = 0; j < sensorDatas[i].size(); ++j) {
			std::string filename = "config/sensor data/toa/" + std::to_string(i) +"_"+std::to_string(j) + ".json";
			sensorDatas[i][j].Write2Json(filename);
		}
	}


}

void TestUWBAOALocalizationMultiStation()
{
	struct LBSData {
		bool isValid;
		int sensorDataId;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), sensorDataId(-1), error(0) {}
		LBSData(const Point2D& p) :isValid(true), sensorDataId(-1), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	//��ȡ��ʵ�ķ��������
	std::vector<Point3D> points;
	std::ifstream instream("tx_position.txt");
	std::string line;
	while (std::getline(instream, line)) {
		if (line.empty()) continue;    // ��������
		std::istringstream iss(line);  // ʹ���ַ���������ÿһ��
		Point3D point;
		if (iss >> point.x >> point.y >> point.z) { // ��ȡ x, y, z ����
			points.push_back(point);   // ����洢��������
		}
		else {
			std::cerr << "���ݸ�ʽ�����޷�����: " << line << std::endl;
		}
	}
	instream.close();
	int txNum = points.size();

	RtLbsType aoaErrorDegree = 6.0;
	RtLbsType powerError = 6.0;
	//��ȡsensorconfig����
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");

	std::vector<LBSData> datas(txNum);

	System* system = nullptr;
	for (int i = 0; i < txNum; ++i) {
		datas[i].truePosition = { points[i].x, points[i].y };
		for (int j = 0; j < 4; ++j) {
			curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = aoaErrorDegree;
			curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = aoaErrorDegree * ONE_DEGEREE;
			curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = 0.2;
			curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
			curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/aoa/" + std::to_string(i) + "_" + std::to_string(j) + ".json";
		}
		curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
		system = new System();
		system->Setup(MODE_LBS);
		system->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = aoaErrorDegree;
		system->Render();
		Point2D targetPosition;
		targetPosition = system->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA);
		datas[i].targetPosition = targetPosition;
		datas[i].error = (targetPosition - datas[i].truePosition).Length();
		std::cout << i << std::endl;
		delete system;
	}

	std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(aoaErrorDegree) + "_powerError_" + std::to_string(powerError) + ".txt";
	std::ofstream outstream(outputFileName);
	for (auto& data : datas) {
		data.Write2File(outstream);
	}
	outstream.close();
}

void TestUWBTOALocalizationMultiStation()
{
	struct LBSData {
		bool isValid;
		int sensorDataId;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), sensorDataId(-1), error(0) {}
		LBSData(const Point2D& p) :isValid(true), sensorDataId(-1), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	//��ȡ��ʵ�ķ��������
	std::vector<Point3D> points;
	std::ifstream instream("tx_position.txt");
	std::string line;
	while (std::getline(instream, line)) {
		if (line.empty()) continue;    // ��������
		std::istringstream iss(line);  // ʹ���ַ���������ÿһ��
		Point3D point;
		if (iss >> point.x >> point.y >> point.z) { // ��ȡ x, y, z ����
			points.push_back(point);   // ����洢��������
		}
		else {
			std::cerr << "���ݸ�ʽ�����޷�����: " << line << std::endl;
		}
	}
	instream.close();
	int txNum = points.size();

	RtLbsType aoaErrorDegree = 6.0;
	RtLbsType powerError = 6.0;
	RtLbsType timeError = 0.3;
	//��ȡsensorconfig����
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");

	std::vector<LBSData> datas(txNum);

	System* system = nullptr;
	for (int i = 200; i < 220; ++i) {
		datas[i].truePosition = { points[i].x, points[i].y };
		for (int j = 0; j < 4; ++j) {
			curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = aoaErrorDegree;
			curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = aoaErrorDegree * ONE_DEGEREE;
			curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = 0.3;
			curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
			curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/toa/" + std::to_string(i) + "_" + std::to_string(j) + ".json";
		}
		curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
		system = new System();
		system->Setup(MODE_LBS);
		system->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = aoaErrorDegree;
		system->Render();
		Point2D targetPosition;
		targetPosition = system->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_TOA);
		datas[i].targetPosition = targetPosition;
		datas[i].error = (targetPosition - datas[i].truePosition).Length();
		std::cout << i << std::endl;
		delete system;
	}

	std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(aoaErrorDegree) +"_timeError_"+std::to_string(timeError) + "_powerError_" + std::to_string(powerError) + ".txt";
	std::ofstream outstream(outputFileName);
	for (auto& data : datas) {
		data.Write2File(outstream);
	}
	outstream.close();
}

void TestUWBAOATOALocalizationMultiStation()
{
	struct LBSData {
		bool isValid;
		int sensorDataId;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), sensorDataId(-1), error(0) {}
		LBSData(const Point2D& p) :isValid(true), sensorDataId(-1), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	//��ȡ��ʵ�ķ��������
	std::vector<Point3D> points;
	std::ifstream instream("tx_position.txt");
	std::string line;
	while (std::getline(instream, line)) {
		if (line.empty()) continue;    // ��������
		std::istringstream iss(line);  // ʹ���ַ���������ÿһ��
		Point3D point;
		if (iss >> point.x >> point.y >> point.z) { // ��ȡ x, y, z ����
			points.push_back(point);   // ����洢��������
		}
		else {
			std::cerr << "���ݸ�ʽ�����޷�����: " << line << std::endl;
		}
	}
	instream.close();
	int txNum = points.size();

	RtLbsType aoaErrorDegree = 6.0;
	RtLbsType powerError = 6.0;
	RtLbsType timeError = 0.3;
	//��ȡsensorconfig����
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");

	std::vector<LBSData> datas(txNum);

	System* system = nullptr;
	for (int i = 0; i < txNum; ++i) {
		datas[i].truePosition = { points[i].x, points[i].y };
		for (int j = 0; j < 4; ++j) {
			curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = aoaErrorDegree;
			curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = aoaErrorDegree * ONE_DEGEREE;
			curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = 0.3;
			curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
			curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/toa/" + std::to_string(i) + "_" + std::to_string(j) + ".json";
		}
		curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
		system = new System();
		system->Setup(MODE_LBS);
		system->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = aoaErrorDegree;
		system->Render();
		Point2D targetPosition;
		targetPosition = system->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA_TOA);
		datas[i].targetPosition = targetPosition;
		datas[i].error = (targetPosition - datas[i].truePosition).Length();
		std::cout << i << std::endl;
		delete system;
	}

	std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(aoaErrorDegree) + "_timeError_" + std::to_string(timeError) + "_powerError_" + std::to_string(powerError) + ".txt";
	std::ofstream outstream(outputFileName);
	for (auto& data : datas) {
		data.Write2File(outstream);
	}
	outstream.close();
}

void TestUWBAOATDOALocalizationMultiStation()
{
	struct LBSData {
		bool isValid;
		int sensorDataId;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), sensorDataId(-1), error(0) {}
		LBSData(const Point2D& p) :isValid(true), sensorDataId(-1), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	//��ȡ��ʵ�ķ��������
	std::vector<Point3D> points;
	std::ifstream instream("tx_position.txt");
	std::string line;
	while (std::getline(instream, line)) {
		if (line.empty()) continue;    // ��������
		std::istringstream iss(line);  // ʹ���ַ���������ÿһ��
		Point3D point;
		if (iss >> point.x >> point.y >> point.z) { // ��ȡ x, y, z ����
			points.push_back(point);   // ����洢��������
		}
		else {
			std::cerr << "���ݸ�ʽ�����޷�����: " << line << std::endl;
		}
	}
	instream.close();
	int txNum = points.size();

	RtLbsType aoaErrorDegree = 6.0;
	RtLbsType powerError = 6.0;
	RtLbsType timeError = 0.3;
	//��ȡsensorconfig����
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");

	std::vector<LBSData> datas(txNum);

	System* system = nullptr;
	for (int i = 0; i < txNum; ++i) {
		datas[i].truePosition = { points[i].x, points[i].y };
		for (int j = 0; j < 4; ++j) {
			curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = aoaErrorDegree;
			curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = aoaErrorDegree * ONE_DEGEREE;
			curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = 0.3;
			curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = powerError - 4.0;
			curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/toa/" + std::to_string(i) + "_" + std::to_string(j) + ".json";
		}
		curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
		system = new System();
		system->Setup(MODE_LBS);
		system->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = 1.0;
		system->Render();
		Point2D targetPosition;
		targetPosition = system->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA_TDOA);
		datas[i].targetPosition = targetPosition;
		datas[i].error = (targetPosition - datas[i].truePosition).Length();
		std::cout << i << std::endl;
		delete system;
	}

	std::string outputFileName = "��λ���ܷ���/UWB��λ/phiError_" + std::to_string(aoaErrorDegree) + "_timeError_" + std::to_string(timeError) + "_powerError_" + std::to_string(powerError) + ".txt";
	std::ofstream outstream(outputFileName);
	for (auto& data : datas) {
		data.Write2File(outstream);
	}
	outstream.close();
}

void TestAOALocalizationMultiStationInGeometryError(int positionId)
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{37,30} };
	std::vector<std::string> postionNames = { "A","B","C" };

	std::vector<RtLbsType> phiDegreeErrors = { 2.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	std::vector<RtLbsType> geometryErrors = { 0.0,0.01,0.02,0.05,0.1,0.2,0.5,1.0,1.5,2.0 };
	int roundTime = 400;

	//Ԥд����
	//��ȡ�������ã�д���Ӧ����ⷽ��
	SimConfig simconfig;
	simconfig.Init("config/sysconfig.json");
	simconfig.m_systemMode = MODE_LBS;
	simconfig.m_lbsConfig.m_lbsMode = LBS_MODE_MPSTSD;
	simconfig.m_lbsConfig.m_lbsMethod = LBS_METHOD_RT_AOA;
	simconfig.m_lbsConfig.m_threadNum = 20;
	simconfig.Writer2Json("config/sysconfig.json");
	//��ȡ��������Ϣ��д���Ӧ���������
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
	for (int j = 0; j < stationNum; ++j) {
		curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeErrors[0];
		curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiDegreeErrors[0] * ONE_DEGEREE;
		curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = 2.0;
		curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_aoa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
	}
	curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");


	System* lbsSystem = nullptr;
	for (auto geometryError : geometryErrors) {
		for (auto phiDegreeError : phiDegreeErrors) {
			for (auto powerError : powerErrors) {
				std::vector<LBSData> datas;
				datas.resize(roundTime);
				
				for (int i = 0; i < roundTime; ++i) {
					datas[i].truePosition = realPositions[positionId];

					//��ȡ������������Ϣ��д���Ӧ�������
					GeometryConfig geometryConfig;
					geometryConfig.Init("config/geometryconfig.json");
					geometryConfig.m_positionError = geometryError;
					geometryConfig.Write2Json("config/geometryconfig.json");

					lbsSystem = new System();
					lbsSystem->Setup(MODE_LBS);
					lbsSystem->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = std::min(0.5, phiDegreeError);
					lbsSystem->Render();
					Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA);
					datas[i].targetPosition = targetPosition;
					datas[i].error = (targetPosition - datas[i].truePosition).Length();
					if (datas[i].error > 10) {
						datas[i].isValid = false;
					}
					std::cout << i << std::endl;
					delete lbsSystem;
				}
				std::stringstream ss;
				ss << "��λ���ܷ���/UWB��λ/" << postionNames[positionId] << "_AOA_geometryError_" << geometryError << ".txt";
				std::ofstream outstream(ss.str());
				for (auto& data : datas) {
					if (data.isValid) {
						data.Write2File(outstream);
					}
				}
				outstream.close();
			}
		}
	}
	
}

void TestTOALocalizationMultiStationInGeometryError(int positionId)
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{37,30} };
	std::vector<std::string> postionNames = { "A","B","C" };

	std::vector<RtLbsType> phiDegreeErrors = { 2.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	std::vector<RtLbsType> timeErrors = { 3.0 };
	std::vector<RtLbsType> geometryErrors = { 0.0,0.01,0.02,0.05,0.1,0.2,0.5,1.0,1.5,2.0 };
	int roundTime = 400;


	//Ԥд����
	//��ȡ�������ã�д���Ӧ����ⷽ��
	SimConfig simconfig;
	simconfig.Init("config/sysconfig.json");
	simconfig.m_systemMode = MODE_LBS;
	simconfig.m_lbsConfig.m_lbsMode = LBS_MODE_MPSTSD;
	simconfig.m_lbsConfig.m_lbsMethod = LBS_METHOD_RT_TOA;
	simconfig.m_lbsConfig.m_threadNum = 10;
	simconfig.Writer2Json("config/sysconfig.json");

	//��ȡ��������Ϣ��д���Ӧ���������
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
	for (int j = 0; j < stationNum; ++j) {
		curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeErrors[0];
		curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiDegreeErrors[0] * ONE_DEGEREE;
		curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = timeErrors[0];
		curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = 2.0;
		curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_toa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
	}
	curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");


	System* lbsSystem = nullptr;
	for (auto geometryError : geometryErrors) {
		for (auto phiDegreeError : phiDegreeErrors) {
			for (auto timeError : timeErrors) {
				for (auto powerError : powerErrors) {
					std::vector<LBSData> datas;
					datas.resize(roundTime);
					//��ȡ��������Ϣ��д���Ӧ���������
					for (int i = 0; i < roundTime; ++i) {
						datas[i].truePosition = realPositions[positionId];
						//��ȡ������������Ϣ��д���Ӧ�������
						GeometryConfig geometryConfig;
						geometryConfig.Init("config/geometryconfig.json");
						geometryConfig.m_positionError = geometryError;
						geometryConfig.Write2Json("config/geometryconfig.json");

						lbsSystem = new System();
						lbsSystem->Setup(MODE_LBS);
						lbsSystem->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = std::min(0.5, phiDegreeError);
						lbsSystem->Render();
						Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_TOA);
						datas[i].targetPosition = targetPosition;
						datas[i].error = (targetPosition - datas[i].truePosition).Length();
						if (datas[i].error > 10) {
							datas[i].isValid = false;
						}
						std::cout << i << std::endl;
						delete lbsSystem;
					}
					std::stringstream ss;
					ss << "��λ���ܷ���/UWB��λ/" << postionNames[positionId] << "_TOA_geometryError_" << geometryError << ".txt";
					std::ofstream outstream(ss.str());
					for (auto& data : datas) {
						if (data.isValid) {
							data.Write2File(outstream);
						}
					}
					outstream.close();
				}
			}
		}
	}
	
}

void TestAOATOALocalizationMultiStationInGeometryError(int positionId)
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{37,30} };
	std::vector<std::string> postionNames = { "A","B","C" };

	std::vector<RtLbsType> phiDegreeErrors = { 2.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	std::vector<RtLbsType> timeErrors = { 3.0 };
	std::vector<RtLbsType> geometryErrors = { 0.0,0.01,0.02,0.05,0.1,0.2,0.5,1.0,1.5,2.0 };
	int roundTime = 400;

	//Ԥд����
	//��ȡ�������ã�д���Ӧ����ⷽ��
	SimConfig simconfig;
	simconfig.Init("config/sysconfig.json");
	simconfig.m_systemMode = MODE_LBS;
	simconfig.m_lbsConfig.m_lbsMode = LBS_MODE_MPSTSD;
	simconfig.m_lbsConfig.m_lbsMethod = LBS_METHOD_RT_AOA_TOA;
	simconfig.m_lbsConfig.m_threadNum = 10;
	simconfig.Writer2Json("config/sysconfig.json");

	//��ȡ��������Ϣ��д���Ӧ���������
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
	for (int j = 0; j < stationNum; ++j) {
		curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeErrors[0];
		curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiDegreeErrors[0] * ONE_DEGEREE;
		curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = timeErrors[0];
		curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = 2.0;
		curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_toa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
	}
	curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");

	System* lbsSystem = nullptr;
	for (auto geometryError : geometryErrors) {
		for (auto phiDegreeError : phiDegreeErrors) {
			for (auto timeError : timeErrors) {
				for (auto powerError : powerErrors) {
					std::vector<LBSData> datas;
					datas.resize(roundTime);
					//��ȡ��������Ϣ��д���Ӧ���������
					for (int i = 0; i < roundTime; ++i) {
						datas[i].truePosition = realPositions[positionId];
						//��ȡ������������Ϣ��д���Ӧ�������
						GeometryConfig geometryConfig;
						geometryConfig.Init("config/geometryconfig.json");
						geometryConfig.m_positionError = geometryError;
						geometryConfig.Write2Json("config/geometryconfig.json");

						lbsSystem = new System();
						lbsSystem->Setup(MODE_LBS);
						lbsSystem->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = std::min(0.5, phiDegreeError);
						lbsSystem->Render();
						Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA_TOA);
						datas[i].targetPosition = targetPosition;
						datas[i].error = (targetPosition - datas[i].truePosition).Length();
						if (datas[i].error > 10) {
							datas[i].isValid = false;
						}
						std::cout << i << std::endl;
						delete lbsSystem;
					}
					std::stringstream ss;
					ss << "��λ���ܷ���/UWB��λ/" << postionNames[positionId] << "_AOATOA_geometryError_" << geometryError << ".txt";
					std::ofstream outstream(ss.str());
					for (auto& data : datas) {
						if (data.isValid) {
							data.Write2File(outstream);
						}
					}
					outstream.close();
				}
			}
		}
	}
}

void TestAOATDOALocalizationMultiStationInGeometryError(int positionId)
{
	struct LBSData {
		bool isValid;
		RtLbsType error;
		Point2D truePosition;
		Point2D targetPosition;
		LBSData() :isValid(true), error(0) {}
		LBSData(const Point2D& p) :isValid(true), error(0), truePosition(p) {}
		void Write2File(std::ofstream& stream) {
			stream << isValid << "\t" << truePosition.x << "\t" << truePosition.y << "\t" << targetPosition.x << "\t" << targetPosition.y << "\t" << error << std::endl;
		}
	};

	int stationNum = 3;																		/** @brief	վ������	*/
	std::vector<Point2D> realPositions = { {14,31},{25,39},{37,30} };
	std::vector<std::string> postionNames = { "A","B","C" };

	std::vector<RtLbsType> phiDegreeErrors = { 2.0 };
	std::vector<RtLbsType> powerErrors = { 6.0 };
	std::vector<RtLbsType> timeErrors = { 3.0 };
	std::vector<RtLbsType> geometryErrors = { 0.0,0.01,0.02,0.05,0.1,0.2,0.5,1.0,1.5,2.0 };
	int roundTime = 400;

	//Ԥд����
	//��ȡ�������ã�д���Ӧ����ⷽ��
	SimConfig simconfig;
	simconfig.Init("config/sysconfig.json");
	simconfig.m_systemMode = MODE_LBS;
	simconfig.m_lbsConfig.m_lbsMode = LBS_MODE_MPSTSD;
	simconfig.m_lbsConfig.m_lbsMethod = LBS_METHOD_RT_AOA_TDOA;
	simconfig.m_lbsConfig.m_threadNum = 10;
	simconfig.Writer2Json("config/sysconfig.json");

	//��ȡ��������Ϣ��д���Ӧ���������
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");
	for (int j = 0; j < stationNum; ++j) {
		curSensorConfig.m_sensorConfigs[j].m_phiDegreeErrorSTD = phiDegreeErrors[0];
		curSensorConfig.m_sensorConfigs[j].m_phiErrorSTD = phiDegreeErrors[0] * ONE_DEGEREE;
		curSensorConfig.m_sensorConfigs[j].m_timeErrorSTD = timeErrors[0];
		curSensorConfig.m_sensorConfigs[j].m_powerErrorSTD = 2.0;
		curSensorConfig.m_sensorConfigs[j].m_sensorDataFileName = "config/sensor data/uwb_sim_toa/tx" + std::to_string(positionId) + "_sensor_" + std::to_string(j) + "_MPSTSD.json";
	}
	curSensorConfig.Write2Json("results/rt/sensor data/MPSTSD/MPSTSD_sensorconfig.json");

	System* lbsSystem = nullptr;
	for (auto geometryError : geometryErrors) {
		for (auto phiDegreeError : phiDegreeErrors) {
			for (auto timeError : timeErrors) {
				for (auto powerError : powerErrors) {
					RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
					std::vector<LBSData> datas;
					datas.resize(roundTime);
					//��ȡ��������Ϣ��д���Ӧ���������
					for (int i = 0; i < roundTime; ++i) {
						datas[i].truePosition = realPositions[positionId];

						//��ȡ������������Ϣ��д���Ӧ�������
						GeometryConfig geometryConfig;
						geometryConfig.Init("config/geometryconfig.json");
						geometryConfig.m_positionError = geometryError;
						geometryConfig.Write2Json("config/geometryconfig.json");

						lbsSystem = new System();
						lbsSystem->Setup(MODE_LBS);
						lbsSystem->m_simConfig.m_lbsConfig.m_rayLaunchHalfTheta = std::min(0.5, phiDegreeError);
						lbsSystem->Render();
						Point2D targetPosition = lbsSystem->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA_TDOA);
						datas[i].targetPosition = targetPosition;
						datas[i].error = (targetPosition - datas[i].truePosition).Length();
						if (datas[i].error > 10) {
							datas[i].isValid = false;
						}
						std::cout << i << std::endl;
						delete lbsSystem;
					}
					std::stringstream ss;
					ss << "��λ���ܷ���/UWB��λ/" << postionNames[positionId] << "_AOATDOA_geometryError_" << geometryError << ".txt";
					std::ofstream outstream(ss.str());
					for (auto& data : datas) {
						if (data.isValid) {
							data.Write2File(outstream);
						}
					}
					outstream.close();
				}
			}
		}
	}
}
