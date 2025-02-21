#ifndef RTLBS_TESTGDOP
#define RTLBS_TESTGDOP

#include "localization/localizationfunction/gdop.h"



void TestGDOP_AOA() {

	Vector2D station1(0.0,0.0);
	Vector2D station2(1000.0, 0.0);

	std::vector<Point2D> stations;
	stations.push_back(station1);
	stations.push_back(station2);

	std::vector<Point2D> points;

	for (int i = 500; i <= 500; i += 2) {
		for (int j = 500; j <= 500; j += 2) {
			points.push_back(Point2D(i, j));
		}
	}

	std::vector<RtLbsType> gdops;

	for (auto point : points) {
		gdops.push_back(ComputeGDOPForAOA(stations, point));
	}

	std::ofstream outfile("gdop_test.txt");
	for (auto val : gdops) {
		outfile << val << std::endl;
	}
	outfile.close();

}



#endif
