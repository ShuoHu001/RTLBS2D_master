#include "managers/logmanager.h"
#include "system.h"


int main(void) {
	LOG_INFO << "GPU加速调试验证" << ENDL;
	System system;
	if (!system.Setup())
		return -1;
	system.Render();
	system.OutputResults();
	return 0;
}