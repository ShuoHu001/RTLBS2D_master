#include "managers/logmanager.h"
#include "system.h"


int main(void) {
	LOG_INFO << "��λ�㷨���ɲ�����֤" << ENDL;
	System system;
	if (!system.Setup())
		return -1;
	system.Render();
	system.PostProcessing();
	system.OutputResults();
	return 0;
}