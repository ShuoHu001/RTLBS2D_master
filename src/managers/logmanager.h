#ifndef RTLBS_LOGMANAGER
#define RTLBS_LOGMANAGER



#include "rtlbs.h"
#include "utility/enum.h"
#include "utility/singleton.h"
#include "utility/define.h"

#define LOG_ERROR LogManager::getInstance().log_with_location(LEVEL_ERROR, __FILE__, __LINE__)
#define LOG_INFO LogManager::getInstance().log(LEVEL_INFO)
#define LOG_WARNING LogManager::getInstance().log_with_location(LEVEL_WARNING, __FILE__, __LINE__)
#define ENDL _ENDL(false);
#define CRASH _ENDL(true);

class _ENDL {
public:
	_ENDL(bool flag) { m_bCrash = flag; }

	//whether crash after the message
	bool m_bCrash;
};

class LogManager :public Singleton<LogManager> {
public:
	LogManager& log(LOGLEVEL level) {
		switch (level) {
		case LEVEL_INFO:
			*this <<COLOR_GREEN<< "[INFO] "<< GetCurrentTime() << " ";
			break;
		case LEVEL_WARNING:
			*this <<COLOR_YELLOW<< "[WARNING] "<< GetCurrentTime() << " " << " ";
			break;
		case LEVEL_ERROR:
			*this <<COLOR_RED<< "[ERROR] "<< GetCurrentTime() << " " <<" ";
			break;
		default:
			break;
		}
		return *this;
	}

	LogManager& operator<<(const std::string& value);

	LogManager& operator<<(const unsigned int value);

	LogManager& operator<<(const int value);

	LogManager& operator<<(const int64_t value);

	LogManager& operator<<(const size_t value);

	LogManager& operator<<(const float value);

	LogManager& operator<<(const double value);

	LogManager& operator<<(const char* value);

	LogManager& operator<<(_ENDL value);

	LogManager& operator<<(std::ostream& (*os)(std::ostream&));

	std::string GetCurrentTime();

	LogManager& log_with_location(LOGLEVEL level, const char* file, int line);


private:
	friend class Singleton<LogManager>;
	std::ofstream m_logFile;
	std::string m_logFileName;

	LogManager();

	~LogManager();

	std::string RemoveColorCode(const std::string& str);

};





#endif
