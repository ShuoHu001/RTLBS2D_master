#include "managers/logmanager.h"
#include "system.h"


int main(void) {
	LOG_INFO << "定位算法集成测试验证" << ENDL;
	System system;
	if (!system.Setup())
		return -1;
	system.Render();
	system.PostProcessing();
	system.OutputResults();
	return 0;
}