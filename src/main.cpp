#include "rtlbs.h"
#include "managers/logmanager.h"
#include "system.h"
#include "managers/randomanager.h"
#include "test/testlbs.h"
#include "test/testgdop.h"
#include "test/testsimilarity.h"
//#include <glog/export.h>
//#include <gflags/gflags.h>
//#include <glog/logging.h>
//#include <ceres/ceres.h>





int main(int argc, char** argv) {

	LOG_INFO << "TDOA-AOA 算法调试验证" << ENDL;
	google::InitGoogleLogging(argv[0]);
	// 禁用所有日志输出
	FLAGS_logtostderr = false;  // 禁止输出到标准错误
	FLAGS_minloglevel = 3;      // 设置日志级别为3，屏蔽ERROR及以下级别的日志


	int max_threads = omp_get_max_threads();
	omp_set_num_threads(20);

	//ResearchMultipathSimilarityInLocalizationInDifferentPlaces();
	//return 0;

	//TestAOATDOALocalizationMultiStationInGeometryError(2);
	//TestAOATDOALocalizationMultiStationInGeometryError(1);
	//TestAOATDOALocalizationMultiStationInGeometryError(2);

	//return 1;

	int mode = 1;									//0为射线追踪，2为定位精度测试，3为定位区域测试

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
		RtLbsType meanAccuracy = 0.0;
		RtLbsType timeConsume = 0;
		int validRoundTime = 0;
		for (int i = 0; i < 1; ++i) {
			System* system = new System();
			if (!system->Setup(MODE_LBS))
				return -1;
			system->Render();
			
			auto start = std::chrono::system_clock::now();

			system->PostProcessing();
			Point2D targetPosition = system->TargetLocalization(LBS_MODE_SPSTMD, LBS_METHOD_RT_AOA_TDOA);
			std::cout << targetPosition.x << "," << targetPosition.y << std::endl;

			//RtLbsType curaccuracy = (targetPosition - Point2D(70, 90)).Length();
			//if (curaccuracy > 200) {
			//	delete system;
			//	continue;
			//}
			//meanAccuracy += curaccuracy;

			auto end = std::chrono::system_clock::now();

			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
			std::cout << duration.count() << std::endl;
			timeConsume += duration.count() / 1e6;

			system->OutputResults();
			delete system;
			validRoundTime++;
		}
		std::cout << meanAccuracy / validRoundTime << std::endl;
	}
	else if (mode == 2) {
		TestTOALocalizationSingleStationInDifferentError(1);
	}
	else if (mode == 3) {
		TestAOATDOALocalizaitonSingleStationErrorInDifferentPlace();
	}
	else if (mode == 4) {
		ResearchMultipathSimilarityInLocalizationInDifferentPlaces();
	}


	
	return 0;
}