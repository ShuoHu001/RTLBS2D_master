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

	//std::vector<RtLbsType> phiDegreeErrors = { 0.1,0.2,0.5,1.0,2.0,3.0,4.0,5.0,6.0 };
	//std::vector<RtLbsType> powerErrors = { 0,1,2,3,4,5,6,7,8,9,10 };
	std::vector<RtLbsType> phiDegreeErrors = { 1.0 };
	std::vector<RtLbsType> powerErrors = { 5 };

	std::string positionName = "T3";
	int roundTime = 100;
	Point2D realPoint = { 26.21,137.33 };

	int totalround = static_cast<int>(phiDegreeErrors.size() * powerErrors.size() * roundTime);

	int roundi = 0;
	for (auto phiDegreeError : phiDegreeErrors) {
		for (auto powerError : powerErrors) {
			RtLbsType phiError = phiDegreeError * ONE_DEGEREE;
			std::vector<LBSData> datas;
			datas.reserve(roundTime);

			//��ȡ��������Ϣ��д���Ӧ���������
			SensorCollectionConfig curSensorCollectionConfig;
			curSensorCollectionConfig.Init("config/SPSTMD_sensorconfig.json");
			curSensorCollectionConfig.m_sensorConfigs[0].m_phiDegreeErrorSTD = phiDegreeError;
			curSensorCollectionConfig.m_sensorConfigs[0].m_powerErrorSTD = powerError;
			curSensorCollectionConfig.Write2Json("config/SPSTMD_sensorconfig.json");
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
		if (lbsSystem->m_scene->m_sensors[0]->m_sensorDataCollection.m_data.size() < 2) {						//���ݲ�����ֻ��һ���ྶ������
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
