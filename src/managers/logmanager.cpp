#include "logmanager.h"

LogManager& LogManager::operator<<(const std::string& value)
{
	if (m_logFile.is_open()) {
		m_logFile << RemoveColorCode(value);
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(const unsigned int value)
{
	if (m_logFile.is_open()) {
		m_logFile << value;
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(const int value)
{
	if (m_logFile.is_open()) {
		m_logFile << value;
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(const int64_t value)
{
	if (m_logFile.is_open()) {
		m_logFile << value;
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(const size_t value)
{
	if (m_logFile.is_open()) {
		m_logFile << value;
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(const float value)
{
	if (m_logFile.is_open()) {
		m_logFile << value;
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(const double value)
{
	if (m_logFile.is_open()) {
		m_logFile << value;
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(const char* value)
{
	if (m_logFile.is_open()) {
		m_logFile << RemoveColorCode(value);
		std::cout << value;
	}
	return *this;
}

LogManager& LogManager::operator<<(_ENDL value)
{
	m_logFile << std::endl;
	if (value.m_bCrash) {
		m_logFile.close();

		std::cout <<std::endl<< COLOR_RESET<< "please check log" << std::endl;
		abort();
	}
	std::cout << COLOR_RESET << std::endl;
	return *this;
}

LogManager& LogManager::operator<<(std::ostream& (*os)(std::ostream&))
{
	if (m_logFile.is_open()) {
		m_logFile << os;
		std::cout << os;
	}
	return *this;
}

std::string LogManager::GetCurrentTime()
{
	auto now = std::chrono::system_clock::now();
	auto time = std::chrono::system_clock::to_time_t(now);
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

	std::stringstream ss;
	ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S.") << std::setfill('0') << std::setw(3) << ms.count();
	return ss.str();
}

LogManager& LogManager::log_without_anything(LOGLEVEL level, const char* file, int line)
{
	log(level);
	return *this;
}

LogManager& LogManager::log_with_location(LOGLEVEL level, const char* file, int line)
{
	log(level);
	*this << "File: " << file << ", Line: " << line << " ";
	return *this;
}

LogManager::LogManager()
{
	m_logFileName = "rtlbs.log";
	m_logFile.open(m_logFileName, std::ios::out | std::ios::app);
}

LogManager::~LogManager()
{
	if (m_logFile.is_open()) {
		m_logFile.close();
	}
}

std::string LogManager::RemoveColorCode(const std::string& str)
{
	std::string result;
	for (auto i = str.begin(); i != str.end(); ++i) {
		if (*i == '\033') { // ESC
			i = std::find(i, str.end(), 'm');
		}
		else {
			result += *i;
		}
	}
	return result;
}
