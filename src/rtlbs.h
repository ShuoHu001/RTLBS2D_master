#ifndef RTLBS__
#define RTLBS__


//include the header file
#include <iostream>
#include <cstring>
#include <string>
#include <sstream>
#include <cmath>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <stack>
#include <vector>
#include <iterator>
#include <array>
#include <filesystem>
#include <unordered_set>


//boost库 用于文件读写
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

// find the correct platform
// currently , Windows and Linux is support
#if defined(_WIN32) || defined(_WIN64)
#define SORT_IN_WINDOWS
#elif defined(__linux__)
#define SORT_IN_LINUX
#elif defined(__APPLE__)
#define SORT_IN_MAC
#endif

//enable debug by default
#define RTLBS_DEBUG

//禁止出现 C4996错误
#pragma warning(disable:4996)
//禁止出现 C4984错误
#pragma warning(disable: 4984)
//禁止出现 C4819错误
#pragma warning(disable: 4819)

#if defined(_MSC_VER) && (_MSC_VER >= 1800) 
#  include <algorithm> // for std::min and std::max 
#endif

#ifdef __cplusplus
#define kdnsjad

#endif

//include Intel MKL library
#include <mkl.h>
#include <mkl_cblas.h>

//include google farmhash
#include <farmhash.h>

//include fftw3 library
#include <fftw3.h>

//include OMP for multi-core accelerate
#include <omp.h>

#endif