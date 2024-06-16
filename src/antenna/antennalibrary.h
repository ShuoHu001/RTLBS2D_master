#ifndef RTLBS_ANTENNALIBRARY
#define RTLBS_ANTENNALIBRARY


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "configuration/library/antennalibraryconfig.h"
#include "antenna.h"


//0-99为内置天线方向图
//100-为外置天线方向图

class AntennaLibrary {
private:
	std::vector<Antenna*> m_antennas;												/** @brief	天线库（外部天线库）	*/
	const static int m_interalAntNum = 5;											/** @brief	当前内部的天线库数量,每增加内置数量时需要进行更新此项	*/

public:
	AntennaLibrary();
	~AntennaLibrary();
	Antenna* GetAntenna(unsigned id) const;											//通过天线id来获取对应的天线对象
	Antenna* GetAntenna(const std::string name) const;								//通过天线名称来获取对应的天线对象
	Antenna* GetAntenna(unsigned id, const Euler& posture) const;					//通过天线Id和姿态来获取对应的天线对象
	Antenna* GetAntenna(const std::string name, const Euler& posture) const;		//通过天线名称和姿态来获取对应的天线对象
	bool Init(const AntennaLibraryConfig& config);									//初始化天线库

private:
	void InitInternalAntennas();													//加载内置的天线方向图
	

};

#endif
