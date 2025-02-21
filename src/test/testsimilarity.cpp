#include "testsimilarity.h"


void ResearchMultipathSimilarityInLocalizationInDifferentPlaces()
{
	//���ͷ���ģʽ
	System* system = new System();
	system->Setup(MODE_RT);
	system->Render();
	system->PostProcessing();

	int resultSize = system->m_result.m_raytracingResult.size();
	std::vector<ReceiverInfo> rxInfos(resultSize);

	for (int i = 0; i < resultSize; ++i) {
		rxInfos[i] = ReceiverInfo(system->m_result.m_raytracingResult[i]);
	}


	//����ྶ���ƶȾ���
	for (int i = 0; i < resultSize; ++i) {
		if (!rxInfos[i].m_isValid) {
			continue;
		}
		for (int j = 0; j < resultSize; ++j) {
			if (!rxInfos[j].m_isValid || j == i) {
				continue;
			}
			if ((rxInfos[i].m_point - rxInfos[j].m_point).Length() > 40) {
				continue;
			}
			rxInfos[i].CanAddToSimilarities_AOA(rxInfos[j]);
			rxInfos[i].CanAddToSimilarities_TOA(rxInfos[j]);
			rxInfos[i].CanAddToSimilarities_AOATDOA(rxInfos[j]);
		}
		std::cout<< "-" << i << "-" << std::endl;
	}

	//���¾���
	for (int i = 0; i < resultSize; ++i) {
		rxInfos[i].UpdateSimilaritiesDistance_AOATDOA();
	}

	std::ofstream stream("��λ���ܷ���/ȫ�����������/errormatrix.txt");
	for (auto& curInfo : rxInfos) {
		curInfo.Write2File(stream);
	}
	stream.close();

	bool output_AOA = false;
	bool output_TOA = true;
	bool output_AOATDOA = false;

	//�ֱ���㲻ͬ����Ķྶ�������ĽǶ�ƫ�ͳ�Ʋ�ͬ�Ƕ�ƫ���Ӧ��λ�þ���

	if (output_AOA = true) {
		std::vector<RtLbsType> phiDegreeErrors = { 0.1,0.5,1.0,2.0,4.0,6.0 };
		for (auto& curPhiError : phiDegreeErrors) {
			std::ofstream newStream("��λ���ܷ���/ȫ�����������/AOA_" + std::to_string(curPhiError) + "_errormatrix.txt");
			for (auto& curInfo : rxInfos) {
				if (!curInfo.m_isValid) {
					newStream << curInfo.m_point.x << "\t" << curInfo.m_point.y << "\t" << 0.0 << std::endl;
					continue;
				}
				RtLbsType curMaxDistance = 0.0;
				RtLbsType curMeanDistance = 0.0;
				curInfo.GetDistanceByAOA(curPhiError * ONE_DEGEREE, curMaxDistance, curMeanDistance);
				newStream << curInfo.m_point.x << "\t" << curInfo.m_point.y << "\t" << curMaxDistance << std::endl;
			}
			newStream.close();
		}
	}

	if (output_TOA == true) {
		std::vector<RtLbsType> timeErrors = { 1,2,5,10,15,20 };
		for (auto& curTimeError : timeErrors) {
			std::ofstream newStream("��λ���ܷ���/ȫ�����������/TOA_" + std::to_string(curTimeError) + "_errormatrix.txt");
			for (auto& curInfo : rxInfos) {
				if (!curInfo.m_isValid) {
					newStream << curInfo.m_point.x << "\t" << curInfo.m_point.y << "\t" << 0.0 << std::endl;
					continue;
				}
				RtLbsType curMaxDistance = 0.0;
				RtLbsType curMeanDistance = 0.0;
				curInfo.GetDistanceByTOA(curTimeError * 1e-9, curMaxDistance, curMeanDistance);
				newStream << curInfo.m_point.x << "\t" << curInfo.m_point.y << "\t" << curMaxDistance << std::endl;
			}
			newStream.close();
		}
	}

	if (output_AOATDOA == true) {
		std::vector<RtLbsType> aoaErrors = { 2.0 };
		std::vector<RtLbsType> timeDiffErrors = { 1,2,5,10,15,20 };
		for (auto curAOAError : aoaErrors) {
			for (auto& curTimeDiffError : timeDiffErrors) {
				std::ofstream newStream("��λ���ܷ���/ȫ�����������/AOA_TDOA_" + std::to_string(curAOAError) + "_" + std::to_string(curTimeDiffError) + "_errormatrix.txt");
				for (auto& curInfo : rxInfos) {
					if (!curInfo.m_isValid) {
						newStream << curInfo.m_point.x << "\t" << curInfo.m_point.y << "\t" << 0.0 << std::endl;
						continue;
					}
					RtLbsType curMaxDistance = 0.0;
					RtLbsType curMeanDistance = 0.0;
					curInfo.GetDistanceByAOATDOA(curAOAError * ONE_DEGEREE, curTimeDiffError * 1e-9, curMaxDistance, curMeanDistance);
					newStream << curInfo.m_point.x << "\t" << curInfo.m_point.y << "\t" << curMeanDistance << std::endl;
				}
				newStream.close();
			}
		}
	}
	
	//���
	std::cout << "finished" << std::endl;

}
