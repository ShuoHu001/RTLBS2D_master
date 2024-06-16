#include "fileoperation.h"


std::string GetFileExtension(const std::string& fileName) {
	return std::filesystem::path(fileName).extension().string();
}

