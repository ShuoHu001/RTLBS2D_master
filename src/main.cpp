#include "managers/logmanager.h"
#include "system.h"
#include "localization/tdoa/tdoasolver.h"
#include "managers/randomanager.h"
#include "test/testlbs.h"
//#include <glog/export.h>
//#include <gflags/gflags.h>
//#include <glog/logging.h>
//#include <ceres/ceres.h>





int main(int argc, char** argv) {
	LOG_INFO << "TDOA 算法调试验证" << ENDL;
	google::InitGoogleLogging(argv[0]);
	//Point2D p1 = { 98,33 };
	//Point2D p2 = { 237.42,33 };
	//Point2D p3 = { 70,74 };

	//Point2D s = { 84,50 };

	//RtLbsType t21 = ((s - p2).Length() - (s - p1).Length()) / LIGHT_VELOCITY_AIR + 1e-9;
	//RtLbsType t31 = ((s - p3).Length() - (s - p1).Length()) / LIGHT_VELOCITY_AIR - 1e-9;


	////RtLbsType t21 = 3.10e-7;
	////RtLbsType t31 = 2.77e-8;

	//GeneralSource* refSource = new GeneralSource();
	//refSource->m_position = p1;
	//refSource->m_sensorData.m_timeDiff = 0;

	//GeneralSource* dataSource1 = new GeneralSource();
	//dataSource1->m_position = p2;
	//dataSource1->m_sensorData.m_timeDiff = t21;

	//GeneralSource* dataSource2 = new GeneralSource();
	//dataSource2->m_position = p3;
	//dataSource2->m_sensorData.m_timeDiff = t31;

	//Point2D outPoint = { 84,50 };

	//TDOASolver solver;
	//solver.SetGeneralSource(refSource, dataSource1, dataSource2);
	//solver.Solving_LS(outPoint);


	int mode = 4;									//0为射线追踪，2为定位精度测试，3为定位区域测试

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
		System system;
		if (!system.Setup(MODE_LBS))
			return -1;
		system.Render();
		system.PostProcessing();
		system.OutputResults();
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