#include "managers/logmanager.h"
#include "system.h"
#include "localization/tdoa/tdoasolver.h"





int main(void) {
	LOG_INFO << "TDOA 算法调试验证" << ENDL;

	//Point2D p1 = { 126,65 };
	//Point2D p2 = { 153,65 };
	//Point2D p3 = { 126,107 };

	//Point2D s = { 122.9,33.2 };

	////RtLbsType t21 = ((s - p2).Length() - (s - p1).Length()) / LIGHT_VELOCITY_AIR + 1e-9;
	////RtLbsType t31 = ((s - p3).Length() - (s - p1).Length()) / LIGHT_VELOCITY_AIR - 1e-9;


	//RtLbsType t21 = 4.045826323571036e-8;
	//RtLbsType t31 = 1.4077989272430767e-7;

	//GeneralSource* refSource = new GeneralSource();
	//refSource->m_position = p1;
	//refSource->m_sensorData.m_timeDiff = 0;

	//GeneralSource* dataSource1 = new GeneralSource();
	//dataSource1->m_position = p2;
	//dataSource1->m_sensorData.m_timeDiff = t21;

	//GeneralSource* dataSource2 = new GeneralSource();
	//dataSource2->m_position = p3;
	//dataSource2->m_sensorData.m_timeDiff = t31;

	//Point2D outPoint = { 0,0 };

	//TDOASolver solver;
	//solver.SetGeneralSource(refSource, dataSource1, dataSource2);
	//solver.Solving_LS(outPoint);


	System system;
	if (!system.Setup())
		return -1;
	system.Render();
	system.PostProcessing();
	system.OutputResults();
	return 0;
}