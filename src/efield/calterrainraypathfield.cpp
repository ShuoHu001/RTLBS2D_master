#include "calterrainraypathfield.h"

Complex CalculateDiffractionEField_PICQUENARD(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	//1-计算发射天线初始场能量
	Vector3D routeStart = path->m_nodes[1]->m_point - path->m_nodes[0]->m_point;		/** @brief	绕射路径起始点向量	*/
	RtLbsType phi = routeStart.Azimuth();									/** @brief	离开发射天线处的方位角	*/
	RtLbsType theta = routeStart.Elevation();								/** @brief	离开发射天线处的俯仰角	*/
	Polarization3D initEField3D;											/** @brief	发射天线处的三维复电场	*/
	CalRadiationField_ForwardRT(txAntenna, power, freq, phi, theta, initEField3D);
	//2-计算地形绕射场衰减
	RtLbsType lamda = LIGHT_VELOCITY_AIR / freq;							/** @brief	波长	*/
	RtLbsType tloss = 0.0;													/** @brief	绕射损耗合计	*/
	RtLbsType dSum = 0.0;													/** @brief	累计的d值（用于排除无效的v值导致的系统冗余计算）	*/
	for (int i = 1; i < path->m_nodes.size() - 1; ++i) {					//去除起始发射点和末尾终止点，循环计算
		RtLbsType h0 = path->m_nodes[i]->m_clearanceHeight;					/** @brief	净空高度		*/
		RtLbsType d1 = path->m_nodes[i]->m_s1 + dSum;						/** @brief	距离前一节点的距离	*/
		RtLbsType d2 = path->m_nodes[i]->m_s2;								/** @brief	距离后一节点的距离	*/
		if (d1 <= EPSILON || d2 <= EPSILON)									/** @brief	若s参数有零值情况，则跳过该点	*/
			continue;
		RtLbsType v = h0 * sqrt(2.0 / lamda * (1.0 / d1 + 1.0 / d2));		/** @brief	菲涅尔-基尔霍夫因子	*/
		if (v <= -1.0) {					//峰值无效-进行长度累计
			dSum += d2;
			continue;
		}
		else
			dSum = 0.0;						//峰值有效-长度累计清零					
		RtLbsType loss = 0.0;												/** @brief	绕射损耗	*/
		if (v <= -1.0) {//判定v值进行损耗测算
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
		if (loss == 0.0) {				//若由于峰峦引起的山峰绕射损耗为0,则表明该峰对接收无影响，可以进行剔除操作

			continue;
		}
		tloss += loss;
	}

	if (tloss == 0) {														//若loss为0 表明该条路径上绕射路径能量无贡献（绕射路径不存在）
		return Complex(0, 0);
	}
	initEField3D.CalculateLOSFieldByLoss(tloss, freq);						//叠加地形绕射损耗后的能量

	RtLbsType pathLength = path->GetPropagationLength();
	initEField3D.CalculateLOSFieldByDistance(pathLength, freq);				//叠加传播距离后的能量
	//3-计算接收场能量
	Vector3D routeEnd = path->m_nodes[path->m_nodes.size() - 1]->m_point - path->m_nodes[path->m_nodes.size() - 2]->m_point;		/** @brief	绕射路径终止点的向量	*/
	RtLbsType phi1 = routeEnd.Azimuth();									/** @brief	到达接收天线处的方位角	*/
	RtLbsType theta1 = routeEnd.Elevation();								/** @brief	到达接收天线处的俯仰角	*/
	Complex receivedEField;													/** @brief	到达接收天线处的复电场	*/
	CalReceivedField(rxAntenna, initEField3D, freq, phi1, theta1, receivedEField);
	return receivedEField;
}

Complex CalculateDiffractionEField_EPSTEIN(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	RtLbsType tloss = 0.0;													/** @brief	总衰减损耗	*/
	//1-计算发射天线初始场能量
	Vector3D routeStart = path->m_nodes[1]->m_point - path->m_nodes[0]->m_point;		/** @brief	绕射路径起始点向量	*/
	RtLbsType phi = routeStart.Azimuth();									/** @brief	离开发射天线处的方位角	*/
	RtLbsType theta = routeStart.Elevation();								/** @brief	离开发射天线处的俯仰角	*/
	Polarization3D initEField3D;											/** @brief	发射天线处的三维复电场	*/
	CalRadiationField_ForwardRT(txAntenna, power, freq, phi, theta, initEField3D);
	RtLbsType f = freq / 1e6;												/** @brief	频率 MHz	*/
	//2-计算地形绕射场衰减
	if (path->m_nodes.size() == 3) {										//若为单个峰，则退化为单峰绕射模型
		RtLbsType& h0 = path->m_nodes[1]->m_clearanceHeight;				/** @brief	净空高度	*/
		RtLbsType d1 = path->m_nodes[1]->m_s1 / 1000;						/** @brief	距离发射点的高度	*/
		RtLbsType d2 = path->m_nodes[1]->m_s2 / 1000;						/** @brief	距离接收点的高度	*/
		RtLbsType d = d1 + d2;												/** @brief	收发天线间水平距离	*/
		RtLbsType v = 2.58e-3 * h0 * sqrt(freq / 1e6 * d / (d1 * d2));		/** @brief	绕射参数	*/
		RtLbsType loss = 6.9 * +20 * log10(sqrt((v - 0.1) * (v - 0.1) + 1) + v - 0.1);		/** @brief	单峰绕射损耗	*/
		tloss += loss;
	}
	else if (path->m_nodes.size() == 4) {									//双峰绕射模型 
		RtLbsType d1 = path->m_nodes[1]->m_s1 / 1000;						//第一个峰距离发射点水平距离 单位km
		RtLbsType d2 = path->m_nodes[2]->m_s1 / 1000;						//第二个峰距离发射点水平距离 单位km
		RtLbsType d3 = path->m_nodes[2]->m_s2 / 1000;						//第二个峰距离接收点水平距离 单位km
		RtLbsType d = (d1 + path->m_nodes[1]->m_s2) / 1000;					//路径总长度 单位km

		const RtLbsType& ht = path->m_nodes.front()->m_point.z;				//发射点高度 单位m
		const RtLbsType& hr = path->m_nodes.back()->m_point.z;				//接收点高度 单位m
		const RtLbsType& h1 = path->m_nodes[1]->m_point.z;					//第一个峰的高度 单位m
		const RtLbsType& h2 = path->m_nodes[2]->m_point.z;					//第二个峰的高度 单位m

		RtLbsType vh1 = h1 + 7.85e-2 * d1 * d2 * EERC_K_INV - (ht * d2 + h2 * d1) / (d1 + d2);		//第一个峰在第发射与第二个峰连线上的净空高度
		RtLbsType vh2 = h2 + 7.85e-2 * d2 * d3 * EERC_K_INV - (h1 * d3 + hr * d2) / (d2 + d3);		//第二个峰在第一个峰与接收连线上的净空高度
		RtLbsType vh11 = h1 + 7.85e-2 * d1 * (d2 + d3) * EERC_K_INV - (ht * (d2 + d3) + hr * d1) / d;	//第一个峰的净空高度
		RtLbsType vh21 = h2 + 7.85e-2 * d3 * (d1 + d2) * EERC_K_INV - (ht * d3 + hr * (d1 + d2)) / d;	//第二个峰的净空高度

		RtLbsType v1 = 2.58e-3 * vh1 * sqrt(f * (d1 + d2) / (d1 * d2));		//绕射系数v1
		RtLbsType v2 = 2.58e-3 * vh2 * sqrt(f * (d2 + d3) / (d2 * d3));		//绕射系数v2
		RtLbsType v3 = 2.58e-3 * vh11 * sqrt(f * d / (d1 * (d2 + d3)));		//绕射系数v3
		RtLbsType v4 = 2.58e-3 * vh21 * sqrt(f * d / (d3 * (d1 + d2)));		//绕射系数v4

		RtLbsType l1 = 6.9 + 20 * log10(sqrt((v1 - 0.1) * (v1 - 0.1) + 1) + v1 - 0.1);		//绕射损耗l1
		RtLbsType l2 = 6.9 + 20 * log10(sqrt((v2 - 0.1) * (v2 - 0.1) + 1) + v2 - 0.1);		//绕射损耗l2
		RtLbsType l3 = 6.9 + 20 * log10(sqrt((v3 - 0.1) * (v3 - 0.1) + 1) + v3 - 0.1);		//绕射损耗l3
		RtLbsType l4 = 6.9 + 20 * log10(sqrt((v4 - 0.1) * (v4 - 0.1) + 1) + v4 - 0.1);		//绕射损耗l4
		RtLbsType lc = 10 * log10((d1 + d2) * (d2 + d3) / d2 * d);							//修正损耗因子

		RtLbsType loss = 0.0;																//由双峰引起的绕射
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
	else {						//多峰绕射等效为复合绕射方法-详见代码
		//1-计算所有v值,寻找最大p值
		const RtLbsType& ht = path->m_nodes.front()->m_point.z;			/** @brief	发射天线高度	*/
		const RtLbsType& hr = path->m_nodes.back()->m_point.z;			/** @brief	接收天线高度	*/
		RtLbsType dTemp = 0.0;											/** @brief	距离临时变量	*/
		RtLbsType d = (path->m_nodes[1]->m_s1 + path->m_nodes[1]->m_s2) / 1000;		/** @brief	发射点与接收点之间的水平距离	*/
		RtLbsType vp = -FLT_MAX;										/** @brief	v 最大值	*/
		RtLbsType dp = 0.0;												/** @brief	v 最大值对应的峰距离发射点的距离 单位km	*/
		RtLbsType hp = 0.0;												/** @brief	v 最大值对应的峰高度 单位m	*/
		int vp_id = 0;													/** @brief	v 最大值对应的ID	*/
		for (int i = 1; i < path->m_nodes.size() - 1; ++i) {
			dTemp += path->m_nodes[i]->m_s1 / 1000;						//累积与发射点的距离，当前峰与发射机之间的水平距离
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
			//2-计算0至p-1处最大v值vt值
			RtLbsType vt = -FLT_MAX;										/** @brief	0-p 之间最大值vt	*/
			dTemp = 0.0;
			for (int j = 1; j < vp_id; ++j) {
				dTemp += path->m_nodes[j]->m_s1 / 1000;						/** @brief	每个节点距离发射点的距离	*/
				RtLbsType h = path->m_nodes[j]->m_point.z + 7.85e-2 * dTemp * (dp - dTemp) * EERC_K_INV - (ht * (dp - dTemp) + hp * dTemp) / dp;
				RtLbsType v = 2.58e-3 * h * sqrt(f * dp / (dTemp * (dp - dTemp)));
				if (vt < v) {												//寻找v最大值
					vt = v;
				}
			}

			//3-计算p+1至n处最大v值vr值
			RtLbsType vr = -FLT_MAX;										/** @brief	p-n之间最大值vr	*/
			for (int k = vp_id + 1; k < path->m_nodes.size() - 1; ++k) {
				RtLbsType dk = path->m_nodes[k]->m_s2 / 1000;				/** @brief	每个峰距离接收天线的距离	*/
				RtLbsType h = path->m_nodes[k]->m_point.z + 7.85e-2 * (dk - dp) * (d - dk) * EERC_K_INV - (hp * (d - dk) + hr * (dk - dp)) / (d - dp);
				RtLbsType v = 2.58e-3 * h * sqrt(f * (d - dp) / ((dk - dp) * (d - dk)));
				if (vr < v) {												//寻找v最大值
					vr = v;
				}
			}

			RtLbsType jvp = 6.9 + 20 * log10(sqrt((vp - 0.1) * (vp - 0.1) + 1) + vp - 0.1);
			RtLbsType jvt = 6.9 + 20 * log10(sqrt((vt - 0.1) * (vt - 0.1) + 1) + vt - 0.1);
			RtLbsType jvr = 6.9 + 20 * log10(sqrt((vr - 0.1) * (vr - 0.1) + 1) + vr - 0.1);
			loss = jvp + (1.0 - exp(-1 * jvp / 6.0)) * (jvt + jvr + 10.0 + 4e-2 * d);
		}
		tloss += loss;														//绕射能量叠加

	}
	if (tloss == 0) {														//若loss为0 表明该条路径上绕射路径能量无贡献（绕射路径不存在）
		return Complex(0, 0);
	}
	initEField3D.CalculateLOSFieldByLoss(tloss, freq);						//叠加地形绕射损耗后的能量

	//3-计算接收场能量
	Vector3D routeEnd = path->m_nodes[path->m_nodes.size() - 1]->m_point - path->m_nodes[path->m_nodes.size() - 2]->m_point;		/** @brief	绕射路径终止点的向量	*/
	RtLbsType phi1 = routeEnd.Azimuth();									/** @brief	到达接收天线处的方位角	*/
	RtLbsType theta1 = routeEnd.Elevation();								/** @brief	到达接收天线处的俯仰角	*/
	Complex receivedEField;													/** @brief	到达接收天线处的复电场	*/
	CalReceivedField(rxAntenna, initEField3D, freq, phi1, theta1, receivedEField);
	return receivedEField;
}

Complex CalculateDiffractionEField_UTD(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	//1-计算发射天线初始场能量
	Vector3D routeStart = path->m_nodes[1]->m_point - path->m_nodes[0]->m_point;		/** @brief	绕射路径起始点向量	*/
	RtLbsType phi = routeStart.Azimuth();									/** @brief	离开发射天线处的方位角	*/
	RtLbsType theta = routeStart.Elevation();								/** @brief	离开发射天线处的俯仰角	*/
	RtLbsType st = routeStart.Length();										/** @brief	路径传播距离 单位m	*/
	Polarization3D initEField3D;											/** @brief	发射天线处的三维复电场	*/
	CalRadiationField_ReverseRT(txAntenna, power, freq, phi, theta, st, initEField3D);	//反向方式计算电磁场

	//2-计算地形绕射节点能量
	for (int i = 1; i < path->m_nodes.size() - 1; ++i) {//遍历节点，求解每个节点的绕射参数
		//获得每个ridge的材质参数
		const Material* mat = path->m_nodes[i]->m_ridge->m_mat;
		CalculateDiffractionField_TerrainUTD(initEField3D, st, path->m_nodes[i - 1]->m_point, path->m_nodes[i]->m_point, path->m_nodes[i + 1]->m_point, path->m_nodes[i]->m_ridge, mat, freq, tranFunctionData); //UTD 方法计算地形绕射
	}

	//3-计算接收场能量
	Vector3D routeEnd = path->m_nodes[path->m_nodes.size() - 1]->m_point - path->m_nodes[path->m_nodes.size() - 2]->m_point;		/** @brief	绕射路径终止点的向量	*/
	RtLbsType phi1 = routeEnd.Azimuth();									/** @brief	到达接收天线处的方位角	*/
	RtLbsType theta1 = routeEnd.Elevation();								/** @brief	到达接收天线处的俯仰角	*/
	Complex receivedEField;													/** @brief	到达接收天线处的复电场	*/
	CalReceivedField(rxAntenna, initEField3D, freq, phi1, theta1, receivedEField);
	return receivedEField;
}

Complex CalculateTerrainDiffractionEField(const TerrainDiffractionPath* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	Complex receivedPower;																					/** @brief	接收功率	*/
	if (path->m_terrainDiffractionMode == DIFFRACTIONMODE_PICQUENARD) {										//Picquenard 损耗计算模式
		receivedPower = CalculateDiffractionEField_PICQUENARD(path, power, freq, txAntenna, rxAntenna);
	}
	else if (path->m_terrainDiffractionMode == DIFFRACTIONMODE_EPSTEIN) {									//EPSTEIN损耗计算模式
		receivedPower = CalculateDiffractionEField_EPSTEIN(path, power, freq, txAntenna, rxAntenna);
	}
	else if (path->m_terrainDiffractionMode == DIFFRACTIONMODE_UTD) {										//UTD损耗计算模式
		receivedPower = CalculateDiffractionEField_UTD(path, power, freq, tranFunctionData, txAntenna, rxAntenna);
	}
	return receivedPower;
}
