#include "managers/logmanager.h"
#include "system.h"


int main(void) {
	LOG_INFO << "GPU���ٵ�����֤" << ENDL;
	System system;
	if (!system.Setup())
		return -1;
	system.Render();
	system.OutputResults();
	return 0;
}