#include "rtlbs.h"
#include "managers/logmanager.h"
#include "system.h"
#include "managers/randomanager.h"
#include "test/testlbs.h"
//#include <glog/export.h>
//#include <gflags/gflags.h>
//#include <glog/logging.h>
//#include <ceres/ceres.h>





int main(int argc, char** argv) {

	LOG_INFO << "TDOA-AOA �㷨������֤" << ENDL;
	google::InitGoogleLogging(argv[0]);
	// ����������־���
	FLAGS_logtostderr = false;  // ��ֹ�������׼����
	FLAGS_minloglevel = 3;      // ������־����Ϊ3������ERROR�����¼������־

	int max_threads = omp_get_max_threads();
	omp_set_num_threads(max_threads - 4);

	int mode = 1;									//0Ϊ����׷�٣�2Ϊ��λ���Ȳ��ԣ�3Ϊ��λ�������

	if (mode == 0) {
		System* system;
		for (int i = 0; i < 1; ++i) {
			system = new System();
			if (!system->Setup(MODE_RT))
				return -1;

			system->Render();


			system->PostProcessing();
			

			system->OutputResults();
			delete system;
		}
		
	}
	else if (mode == 1) {
		RtLbsType timeConsume = 0;
		for (int i = 0; i < 100; ++i) {
			System* system = new System();
			if (!system->Setup(MODE_LBS))
				return -1;
			system->Render();
			
			auto start = std::chrono::system_clock::now();

			system->PostProcessing();
			Point2D targetPosition = system->TargetLocalization(LBS_MODE_MPSTSD, LBS_METHOD_RT_AOA_TDOA);
			std::cout << targetPosition.x << "," << targetPosition.y << std::endl;

			auto end = std::chrono::system_clock::now();

			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
			std::cout << duration.count() << std::endl;
			timeConsume += duration.count() / 1e6;

			system->OutputResults();
			delete system;
		}
		std::cout << timeConsume / 100 << std::endl;
	}
	else if (mode == 2) {
		TestAOALocalizationSingleStationInDifferentError();
	}
	else if (mode == 3) {
		TestAOALocalizaitonSingleStationErrorInDifferentPlace();
	}
	else if (mode == 4) {
		ResearchMultipathSimilarityInLocalizationInDifferentPlaces();
	}


	

	
	return 0;
}