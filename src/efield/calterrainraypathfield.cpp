#include "calterrainraypathfield.h"

Complex CalculateDiffractionEField_PICQUENARD(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	//1-���㷢�����߳�ʼ������
	Vector3D routeStart = path->m_nodes[1]->m_point - path->m_nodes[0]->m_point;		/** @brief	����·����ʼ������	*/
	RtLbsType phi = routeStart.Azimuth();									/** @brief	�뿪�������ߴ��ķ�λ��	*/
	RtLbsType theta = routeStart.Elevation();								/** @brief	�뿪�������ߴ��ĸ�����	*/
	Polarization3D initEField3D;											/** @brief	�������ߴ�����ά���糡	*/
	CalRadiationField_ForwardRT(txAntenna, power, freq, phi, theta, initEField3D);
	//2-����������䳡˥��
	RtLbsType lamda = LIGHT_VELOCITY_AIR / freq;							/** @brief	����	*/
	RtLbsType tloss = 0.0;													/** @brief	������ĺϼ�	*/
	RtLbsType dSum = 0.0;													/** @brief	�ۼƵ�dֵ�������ų���Ч��vֵ���µ�ϵͳ������㣩	*/
	for (int i = 1; i < path->m_nodes.size() - 1; ++i) {					//ȥ����ʼ������ĩβ��ֹ�㣬ѭ������
		RtLbsType h0 = path->m_nodes[i]->m_clearanceHeight;					/** @brief	���ո߶�		*/
		RtLbsType d1 = path->m_nodes[i]->m_s1 + dSum;						/** @brief	����ǰһ�ڵ�ľ���	*/
		RtLbsType d2 = path->m_nodes[i]->m_s2;								/** @brief	�����һ�ڵ�ľ���	*/
		if (d1 <= EPSILON || d2 <= EPSILON)									/** @brief	��s��������ֵ������������õ�	*/
			continue;
		RtLbsType v = h0 * sqrt(2.0 / lamda * (1.0 / d1 + 1.0 / d2));		/** @brief	������-������������	*/
		if (v <= -1.0) {					//��ֵ��Ч-���г����ۼ�
			dSum += d2;
			continue;
		}
		else
			dSum = 0.0;						//��ֵ��Ч-�����ۼ�����					
		RtLbsType loss = 0.0;												/** @brief	�������	*/
		if (v <= -1.0) {//�ж�vֵ������Ĳ���
			loss = 0.0;
		}
		else if (v > -1.0 && v <= 0.0) {
			loss = 20 * log10((0.5 - 0.6 * v));
		}
		else if (v > 0.0 && v <= 1.0) {
			loss = 20 * log10(0.5 * exp(-0.95 * v));
		}
		else if (v > 1.0 && v <= 2.4) {
			loss = 20 * log10(0.4 - sqrt(0.1184 - (0.38 - 0.1 * v) * (0.38 - 0.1 * v)));
		}
		else if (v > 2.4) {
			loss = 20 * log10(0.23 / v);
		}
		if (loss == 0.0) {				//�����ڷ��������ɽ���������Ϊ0,������÷�Խ�����Ӱ�죬���Խ����޳�����

			continue;
		}
		tloss += loss;
	}

	if (tloss == 0) {														//��lossΪ0 ��������·��������·�������޹��ף�����·�������ڣ�
		return Complex(0, 0);
	}
	initEField3D.CalculateLOSFieldByLoss(tloss, freq);						//���ӵ���������ĺ������

	RtLbsType pathLength = path->GetPropagationLength();
	initEField3D.CalculateLOSFieldByDistance(pathLength, freq);				//���Ӵ�������������
	//3-������ճ�����
	Vector3D routeEnd = path->m_nodes[path->m_nodes.size() - 1]->m_point - path->m_nodes[path->m_nodes.size() - 2]->m_point;		/** @brief	����·����ֹ�������	*/
	RtLbsType phi1 = routeEnd.Azimuth();									/** @brief	����������ߴ��ķ�λ��	*/
	RtLbsType theta1 = routeEnd.Elevation();								/** @brief	����������ߴ��ĸ�����	*/
	Complex receivedEField;													/** @brief	����������ߴ��ĸ��糡	*/
	CalReceivedField(rxAntenna, initEField3D, freq, phi1, theta1, receivedEField);
	return receivedEField;
}

Complex CalculateDiffractionEField_EPSTEIN(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	RtLbsType tloss = 0.0;													/** @brief	��˥�����	*/
	//1-���㷢�����߳�ʼ������
	Vector3D routeStart = path->m_nodes[1]->m_point - path->m_nodes[0]->m_point;		/** @brief	����·����ʼ������	*/
	RtLbsType phi = routeStart.Azimuth();									/** @brief	�뿪�������ߴ��ķ�λ��	*/
	RtLbsType theta = routeStart.Elevation();								/** @brief	�뿪�������ߴ��ĸ�����	*/
	Polarization3D initEField3D;											/** @brief	�������ߴ�����ά���糡	*/
	CalRadiationField_ForwardRT(txAntenna, power, freq, phi, theta, initEField3D);
	RtLbsType f = freq / 1e6;												/** @brief	Ƶ�� MHz	*/
	//2-����������䳡˥��
	if (path->m_nodes.size() == 3) {										//��Ϊ�����壬���˻�Ϊ��������ģ��
		RtLbsType& h0 = path->m_nodes[1]->m_clearanceHeight;				/** @brief	���ո߶�	*/
		RtLbsType d1 = path->m_nodes[1]->m_s1 / 1000;						/** @brief	���뷢���ĸ߶�	*/
		RtLbsType d2 = path->m_nodes[1]->m_s2 / 1000;						/** @brief	������յ�ĸ߶�	*/
		RtLbsType d = d1 + d2;												/** @brief	�շ����߼�ˮƽ����	*/
		RtLbsType v = 2.58e-3 * h0 * sqrt(freq / 1e6 * d / (d1 * d2));		/** @brief	�������	*/
		RtLbsType loss = 6.9 * +20 * log10(sqrt((v - 0.1) * (v - 0.1) + 1) + v - 0.1);		/** @brief	�����������	*/
		tloss += loss;
	}
	else if (path->m_nodes.size() == 4) {									//˫������ģ�� 
		RtLbsType d1 = path->m_nodes[1]->m_s1 / 1000;						//��һ������뷢���ˮƽ���� ��λkm
		RtLbsType d2 = path->m_nodes[2]->m_s1 / 1000;						//�ڶ�������뷢���ˮƽ���� ��λkm
		RtLbsType d3 = path->m_nodes[2]->m_s2 / 1000;						//�ڶ����������յ�ˮƽ���� ��λkm
		RtLbsType d = (d1 + path->m_nodes[1]->m_s2) / 1000;					//·���ܳ��� ��λkm

		const RtLbsType& ht = path->m_nodes.front()->m_point.z;				//�����߶� ��λm
		const RtLbsType& hr = path->m_nodes.back()->m_point.z;				//���յ�߶� ��λm
		const RtLbsType& h1 = path->m_nodes[1]->m_point.z;					//��һ����ĸ߶� ��λm
		const RtLbsType& h2 = path->m_nodes[2]->m_point.z;					//�ڶ�����ĸ߶� ��λm

		RtLbsType vh1 = h1 + 7.85e-2 * d1 * d2 * EERC_K_INV - (ht * d2 + h2 * d1) / (d1 + d2);		//��һ�����ڵڷ�����ڶ����������ϵľ��ո߶�
		RtLbsType vh2 = h2 + 7.85e-2 * d2 * d3 * EERC_K_INV - (h1 * d3 + hr * d2) / (d2 + d3);		//�ڶ������ڵ�һ��������������ϵľ��ո߶�
		RtLbsType vh11 = h1 + 7.85e-2 * d1 * (d2 + d3) * EERC_K_INV - (ht * (d2 + d3) + hr * d1) / d;	//��һ����ľ��ո߶�
		RtLbsType vh21 = h2 + 7.85e-2 * d3 * (d1 + d2) * EERC_K_INV - (ht * d3 + hr * (d1 + d2)) / d;	//�ڶ�����ľ��ո߶�

		RtLbsType v1 = 2.58e-3 * vh1 * sqrt(f * (d1 + d2) / (d1 * d2));		//����ϵ��v1
		RtLbsType v2 = 2.58e-3 * vh2 * sqrt(f * (d2 + d3) / (d2 * d3));		//����ϵ��v2
		RtLbsType v3 = 2.58e-3 * vh11 * sqrt(f * d / (d1 * (d2 + d3)));		//����ϵ��v3
		RtLbsType v4 = 2.58e-3 * vh21 * sqrt(f * d / (d3 * (d1 + d2)));		//����ϵ��v4

		RtLbsType l1 = 6.9 + 20 * log10(sqrt((v1 - 0.1) * (v1 - 0.1) + 1) + v1 - 0.1);		//�������l1
		RtLbsType l2 = 6.9 + 20 * log10(sqrt((v2 - 0.1) * (v2 - 0.1) + 1) + v2 - 0.1);		//�������l2
		RtLbsType l3 = 6.9 + 20 * log10(sqrt((v3 - 0.1) * (v3 - 0.1) + 1) + v3 - 0.1);		//�������l3
		RtLbsType l4 = 6.9 + 20 * log10(sqrt((v4 - 0.1) * (v4 - 0.1) + 1) + v4 - 0.1);		//�������l4
		RtLbsType lc = 10 * log10((d1 + d2) * (d2 + d3) / d2 * d);							//�����������

		RtLbsType loss = 0.0;																//��˫�����������
		if (l1 >= 15 && l2 >= 15) {
			loss = l1 + l2 + lc;
		}
		else if (l2 < 15 && l3 >= 15) {
			loss = l2 + l3;
		}
		else if (l1 < 15 && l4 >= 15) {
			loss = l1 + l4;
		}
		tloss += loss;

	}
	else {						//��������ЧΪ�������䷽��-�������
		//1-��������vֵ,Ѱ�����pֵ
		const RtLbsType& ht = path->m_nodes.front()->m_point.z;			/** @brief	�������߸߶�	*/
		const RtLbsType& hr = path->m_nodes.back()->m_point.z;			/** @brief	�������߸߶�	*/
		RtLbsType dTemp = 0.0;											/** @brief	������ʱ����	*/
		RtLbsType d = (path->m_nodes[1]->m_s1 + path->m_nodes[1]->m_s2) / 1000;		/** @brief	���������յ�֮���ˮƽ����	*/
		RtLbsType vp = -FLT_MAX;										/** @brief	v ���ֵ	*/
		RtLbsType dp = 0.0;												/** @brief	v ���ֵ��Ӧ�ķ���뷢���ľ��� ��λkm	*/
		RtLbsType hp = 0.0;												/** @brief	v ���ֵ��Ӧ�ķ�߶� ��λm	*/
		int vp_id = 0;													/** @brief	v ���ֵ��Ӧ��ID	*/
		for (int i = 1; i < path->m_nodes.size() - 1; ++i) {
			dTemp += path->m_nodes[i]->m_s1 / 1000;						//�ۻ��뷢���ľ��룬��ǰ���뷢���֮���ˮƽ����
			RtLbsType h = path->m_nodes[i]->m_point.z + 7.85e-2 * dTemp * (d - dTemp) * EERC_K_INV - (ht * (d - dTemp) + hr * dTemp) / d;
			RtLbsType v = 2.58e-3 * h * sqrt(f * d / (dTemp * (d - dTemp)));
			if (v > vp) {
				vp = v;
				dp = dTemp;
				hp = path->m_nodes[i]->m_point.z;
				vp_id = i;
			}
		}

		RtLbsType loss = 0.0;
		if (vp <= -0.78)
			loss = 0.0;
		else {
			//2-����0��p-1�����vֵvtֵ
			RtLbsType vt = -FLT_MAX;										/** @brief	0-p ֮�����ֵvt	*/
			dTemp = 0.0;
			for (int j = 1; j < vp_id; ++j) {
				dTemp += path->m_nodes[j]->m_s1 / 1000;						/** @brief	ÿ���ڵ���뷢���ľ���	*/
				RtLbsType h = path->m_nodes[j]->m_point.z + 7.85e-2 * dTemp * (dp - dTemp) * EERC_K_INV - (ht * (dp - dTemp) + hp * dTemp) / dp;
				RtLbsType v = 2.58e-3 * h * sqrt(f * dp / (dTemp * (dp - dTemp)));
				if (vt < v) {												//Ѱ��v���ֵ
					vt = v;
				}
			}

			//3-����p+1��n�����vֵvrֵ
			RtLbsType vr = -FLT_MAX;										/** @brief	p-n֮�����ֵvr	*/
			for (int k = vp_id + 1; k < path->m_nodes.size() - 1; ++k) {
				RtLbsType dk = path->m_nodes[k]->m_s2 / 1000;				/** @brief	ÿ�������������ߵľ���	*/
				RtLbsType h = path->m_nodes[k]->m_point.z + 7.85e-2 * (dk - dp) * (d - dk) * EERC_K_INV - (hp * (d - dk) + hr * (dk - dp)) / (d - dp);
				RtLbsType v = 2.58e-3 * h * sqrt(f * (d - dp) / ((dk - dp) * (d - dk)));
				if (vr < v) {												//Ѱ��v���ֵ
					vr = v;
				}
			}

			RtLbsType jvp = 6.9 + 20 * log10(sqrt((vp - 0.1) * (vp - 0.1) + 1) + vp - 0.1);
			RtLbsType jvt = 6.9 + 20 * log10(sqrt((vt - 0.1) * (vt - 0.1) + 1) + vt - 0.1);
			RtLbsType jvr = 6.9 + 20 * log10(sqrt((vr - 0.1) * (vr - 0.1) + 1) + vr - 0.1);
			loss = jvp + (1.0 - exp(-1 * jvp / 6.0)) * (jvt + jvr + 10.0 + 4e-2 * d);
		}
		tloss += loss;														//������������

	}
	if (tloss == 0) {														//��lossΪ0 ��������·��������·�������޹��ף�����·�������ڣ�
		return Complex(0, 0);
	}
	initEField3D.CalculateLOSFieldByLoss(tloss, freq);						//���ӵ���������ĺ������

	//3-������ճ�����
	Vector3D routeEnd = path->m_nodes[path->m_nodes.size() - 1]->m_point - path->m_nodes[path->m_nodes.size() - 2]->m_point;		/** @brief	����·����ֹ�������	*/
	RtLbsType phi1 = routeEnd.Azimuth();									/** @brief	����������ߴ��ķ�λ��	*/
	RtLbsType theta1 = routeEnd.Elevation();								/** @brief	����������ߴ��ĸ�����	*/
	Complex receivedEField;													/** @brief	����������ߴ��ĸ��糡	*/
	CalReceivedField(rxAntenna, initEField3D, freq, phi1, theta1, receivedEField);
	return receivedEField;
}

Complex CalculateDiffractionEField_UTD(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	//1-���㷢�����߳�ʼ������
	Vector3D routeStart = path->m_nodes[1]->m_point - path->m_nodes[0]->m_point;		/** @brief	����·����ʼ������	*/
	RtLbsType phi = routeStart.Azimuth();									/** @brief	�뿪�������ߴ��ķ�λ��	*/
	RtLbsType theta = routeStart.Elevation();								/** @brief	�뿪�������ߴ��ĸ�����	*/
	RtLbsType st = routeStart.Length();										/** @brief	·���������� ��λm	*/
	Polarization3D initEField3D;											/** @brief	�������ߴ�����ά���糡	*/
	CalRadiationField_ReverseRT(txAntenna, power, freq, phi, theta, st, initEField3D);	//����ʽ�����ų�

	//2-�����������ڵ�����
	for (int i = 1; i < path->m_nodes.size() - 1; ++i) {//�����ڵ㣬���ÿ���ڵ���������
		//���ÿ��ridge�Ĳ��ʲ���
		const Material* mat = path->m_nodes[i]->m_ridge->m_mat;
		CalculateDiffractionField_TerrainUTD(initEField3D, st, path->m_nodes[i - 1]->m_point, path->m_nodes[i]->m_point, path->m_nodes[i + 1]->m_point, path->m_nodes[i]->m_ridge, mat, freq, tranFunctionData); //UTD ���������������
	}

	//3-������ճ�����
	Vector3D routeEnd = path->m_nodes[path->m_nodes.size() - 1]->m_point - path->m_nodes[path->m_nodes.size() - 2]->m_point;		/** @brief	����·����ֹ�������	*/
	RtLbsType phi1 = routeEnd.Azimuth();									/** @brief	����������ߴ��ķ�λ��	*/
	RtLbsType theta1 = routeEnd.Elevation();								/** @brief	����������ߴ��ĸ�����	*/
	Complex receivedEField;													/** @brief	����������ߴ��ĸ��糡	*/
	CalReceivedField(rxAntenna, initEField3D, freq, phi1, theta1, receivedEField);
	return receivedEField;
}

Complex CalculateTerrainDiffractionEField(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	Complex receivedPower;																					/** @brief	���չ���	*/
	if (path->m_terrainDiffractionMode == DIFFRACTIONMODE_PICQUENARD) {										//Picquenard ��ļ���ģʽ
		receivedPower = CalculateDiffractionEField_PICQUENARD(path, power, freq, txAntenna, rxAntenna);
	}
	else if (path->m_terrainDiffractionMode == DIFFRACTIONMODE_EPSTEIN) {									//EPSTEIN��ļ���ģʽ
		receivedPower = CalculateDiffractionEField_EPSTEIN(path, power, freq, txAntenna, rxAntenna);
	}
	else if (path->m_terrainDiffractionMode == DIFFRACTIONMODE_UTD) {										//UTD��ļ���ģʽ
		receivedPower = CalculateDiffractionEField_UTD(path, power, freq, tranFunctionData, txAntenna, rxAntenna);
	}
	return receivedPower;
}
