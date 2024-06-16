#ifndef RTLBS_TRANFUNCTIONDATA
#define RTLBS_TRANFUNCTIONDATA

#include "rtlbs.h"
#include "utility/define.h"
#include "math/complex.h"


#ifndef TRANFUNCTION_DLL
#define TRANFUNCTION_DLL __declspec(dllimport)
#else
#define TRANFUNCTION_DLL __declspec(dllexport)
#endif

TRANFUNCTION_DLL std::vector<Complex> GetTranFunctionData();


#endif
