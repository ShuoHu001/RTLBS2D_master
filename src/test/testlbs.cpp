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

	std::vector<RtLbsType> phiDegreeErrors = { 0.1,0.2,0.5,1.0,2.0,3.0,4.0,5.0,6.0 };
	std::vector<RtLbsType> powerErrors = { 0,1,2,3,4,5,6,7,8,9,10 };

	std::string positionName = "A1";
	int roundTime = 1000;
	Point2D realPoint = { 76,56 };

	int totalround = static_cast<int>(phiDegreeErrors.size() * powerErrors.size() * roundTime);

	int roundi = 0;
	for (auto phiDegreeError : phiDegreeErrors) {
		for (auto powerError : powerErrors) {
			RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
			std::vector<LBSData> datas;
			datas.reserve(roundTime);

			//读取传感器信息，写入对应的误差数据
			SensorCollectionConfig curSensorCollectionConfig;
			curSensorCollectionConfig.Init("config/SPSTMD_sensorconfig.json");
			curSensorCollectionConfig.m_sensorConfigs[0].m_phiDegreeErrorSTD = phiDegreeError;
			curSensorCollectionConfig.m_sensorConfigs[0].m_powerErrorSTD = powerError;
			curSensorCollectionConfig.Write2Json("config/SPSTMD_sensorconfig.json");
			System* system;
			for (int i = 0; i < roundTime; ++i) {
				std::cout << roundi++ / static_cast<RtLbsType>(totalround) * 100 << std::endl;			//汇报进度

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
				curData.targetPosition = curTargetPoint;
				curData.error = error;
				datas.push_back(curData);
				delete system;
			}

			//计算平均误差
			RtLbsType errorMean = 0.0;
			for (auto& data : datas) {
				errorMean += data.error;
			}
			errorMean /= roundTime;

			//写入数据文件
			std::stringstream ss;
			ss << "定位性能分析/单点仿真分析/" << positionName << "_phiError_" << phiDegreeError << "_powerError_" << powerError << ".txt";
			std::ofstream outFile(ss.str());
			for (auto& data : datas) {
				data.Write2File(outFile);
			}
			outFile.close();
			LOG_INFO << errorMean << std::endl;
		}
	}
	LOG_INFO << "计算完成" << ENDL;
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

	bool isAlreadyGenerateSensorData = true;				/** @brief	是否已经生成传感器数据	*/

	//生成定位坐标集
	std::vector<LBSData*> datas;
	RtLbsType xmin = 28;
	RtLbsType xmax = 135;
	RtLbsType ymin = 28;
	RtLbsType ymax = 82;
	RtLbsType gap = 2;
	for (RtLbsType y = ymin; y < ymax; y+=gap) {
		for (RtLbsType x = xmin; x < xmax; x += gap) {
			LBSData* lbsData = new LBSData(Point2D(x, y));
			datas.push_back(lbsData);
		}
	}
	
	System* rtSystem = new System();
	rtSystem->Setup(MODE_RT);
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
	txCollectionConfig.Write2Json("config/transmitterconfig.json");		//生成定位数据


	//------------------------------------------传感器数据生成----------------------------------------------------------------------------------------
	if (!isAlreadyGenerateSensorData) {
		rtSystem = new System();
		rtSystem->Setup(MODE_RT);
		rtSystem->Render();
		rtSystem->PostProcessing();
		rtSystem->OutputResults();
		delete rtSystem;
	}
	//-----------------------------------------------------------------------------------------------------------------------------------------------

	//修改生成传感器数据的文件
	SimConfig curSimConfig;
	curSimConfig.Init("config/sysconfig.json");
	curSimConfig.SetSeneorConfigFile("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
	curSimConfig.Writer2Json("config/sysconfig.json");

	RtLbsType phiDegreeError = 8.0;
	RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
	RtLbsType powerError = 0.0;
	//循环计算定位误差
	SensorCollectionConfig curSensorConfig;
	curSensorConfig.Init("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
	std::string sensorDataFileName;
	
	#pragma omp parallel for num_threads(10)
	int round = 0;
	for (auto& data : datas) {
		std::cout << round++ << std::endl;
		if (!data->isValid) {
			continue;
		}
		//if (data->truePosition.x != 28 || data->truePosition.y != 78) {
		//	continue;
		//}
		sensorDataFileName = "results/rt/sensor data/SPSTMD/tx" + std::to_string(data->sensorDataId) + "_SPSTMD.json";
		curSensorConfig.m_sensorConfigs[0].m_sensorDataFileName = sensorDataFileName;
		curSensorConfig.Write2Json("results/rt/sensor data/SPSTMD/SPSTMD_sensorconfig.json");
		System* lbsSystem = new System();
		
		lbsSystem->Setup(MODE_LBS);
		if (lbsSystem->m_scene->m_sensors[0]->m_sensorDataCollection.m_data.size() < 2) {						//数据不合理，只有一条多径，错误
			data->isValid = false;
			delete lbsSystem;
			continue;
		}
		lbsSystem->m_scene->m_sensors[0]->m_phiErrorSTD = phiError;
		lbsSystem->m_scene->m_sensors[0]->m_powerErrorSTD = powerError;
		lbsSystem->Render();
		Point2D targetPosition;
		targetPosition = lbsSystem->TargetLocalization(LBS_MODE_SPSTMD, LBS_METHOD_RT_AOA);
		delete lbsSystem;
		data->targetPosition = targetPosition;
		data->error = (targetPosition - data->truePosition).Length();
	}

	//将数据写入至文件中
	std::string outputFileName = "定位性能分析/全域仿真分析/phiError_" + std::to_string(phiDegreeError) + "_powerError_" + std::to_string(powerError) + ".txt";
	std::ofstream outstream(outputFileName);
	for (auto& data : datas) {
		data->Write2File(outstream);
	}
	outstream.close();

	std::cout << "计算完成" << std::endl;
}

void ResearchMultipathSimilarityInLocalizationInDifferentPlaces()
{
	//面型仿真模式
	System* system = new System();
	system->Setup(MODE_RT);
	system->Render();
	system->PostProcessing();

	int resultSize = system->m_result.m_raytracingResult.size();
	std::vector<ReceiverInfo*> rxInfos(resultSize);

	for (int i = 0; i < resultSize; ++i) {
		rxInfos[i] = new ReceiverInfo(system->m_result.m_raytracingResult[i]);
	}

	//构造多径相似度矩阵
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

	//更新距离
	for (int i = 0; i < resultSize; ++i) {
		ReceiverInfo* curInfo = rxInfos[i];
		curInfo->UpdateSimilaritiesDistance();
	}

	std::ofstream stream("定位性能分析/全域误差矩阵分析/errormatrix.txt");
	for (auto& curInfo : rxInfos) {
		curInfo->Write2File(stream);
	}
	//完成
	std::cout << "finished" << std::endl;


}
