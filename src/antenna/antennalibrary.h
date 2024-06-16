#ifndef RTLBS_ANTENNALIBRARY
#define RTLBS_ANTENNALIBRARY


#include "rtlbs.h"
#include "utility/define.h"
#include "utility/serializable.h"
#include "configuration/library/antennalibraryconfig.h"
#include "antenna.h"


//0-99Ϊ�������߷���ͼ
//100-Ϊ�������߷���ͼ

class AntennaLibrary {
private:
	std::vector<Antenna*> m_antennas;												/** @brief	���߿⣨�ⲿ���߿⣩	*/
	const static int m_interalAntNum = 5;											/** @brief	��ǰ�ڲ������߿�����,ÿ������������ʱ��Ҫ���и��´���	*/

public:
	AntennaLibrary();
	~AntennaLibrary();
	Antenna* GetAntenna(unsigned id) const;											//ͨ������id����ȡ��Ӧ�����߶���
	Antenna* GetAntenna(const std::string name) const;								//ͨ��������������ȡ��Ӧ�����߶���
	Antenna* GetAntenna(unsigned id, const Euler& posture) const;					//ͨ������Id����̬����ȡ��Ӧ�����߶���
	Antenna* GetAntenna(const std::string name, const Euler& posture) const;		//ͨ���������ƺ���̬����ȡ��Ӧ�����߶���
	bool Init(const AntennaLibraryConfig& config);									//��ʼ�����߿�

private:
	void InitInternalAntennas();													//�������õ����߷���ͼ
	

};

#endif
