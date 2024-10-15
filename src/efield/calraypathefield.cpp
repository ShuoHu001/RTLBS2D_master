#include "calraypathefield.h"

Polarization3D CalculateStrengthField3D(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna)
{
	if (path->m_nodes.size() < 2)		//不满足电磁计算的条件
		return Polarization3D();
	Polarization3D efield;								/** @brief	初始三维极化电磁场	*/
	PathNode3D* curNode = nullptr;
	PathNode3D* preNode = nullptr;
	PathNode3D* nextNode = nullptr;
	preNode = path->m_nodes[0];
	curNode = path->m_nodes[1];
	//首节点--计算辐射场
	Vector3D pc = curNode->m_point - preNode->m_point;												/** @brief	前一节点指向当前节点的向量	*/
	RtLbsType st = pc.Length();																		/** @brief	累积距离	*/
	CalRadiationField_ReverseRT(txAntenna, power, freq, pc.Azimuth(), pc.Elevation(), st, efield);		/** @brief	计算初始辐射场	*/
	//计算中间节点的辐射场
	for (int i = 1; i < path->m_nodes.size() - 1; ++i) {
		if (i != 1)										//当且仅当在首节点时不会进行前节点替换操作
			preNode = curNode;							//更新前一节点
		curNode = path->m_nodes[i];						//更新当前节点
		nextNode = path->m_nodes[i + 1];					//更新下一节点
		Material* matCurNode = curNode->m_mat;
		if (curNode->m_type == NODE_REFL) {
			CalculateReflectionField_ReverseRT(efield, st, preNode->m_point, curNode->m_point, nextNode->m_point, matCurNode, freq);
		}
		else if (curNode->m_type == NODE_TRANIN || curNode->m_type == NODE_TRANOUT) {
			CalculateTransmissionField_ReverseRT(efield, st, preNode->m_point, curNode->m_point, nextNode->m_point, matCurNode, freq);
		}
		else if (curNode->m_type == NODE_ETRANIN) {
			CalculateStraightTransmissionField_ReverseRT(efield, st, curNode->m_point, nextNode->m_point, matCurNode, freq);
		}
		else if (curNode->m_type == NODE_ETRANOUT) {
			efield.CalculateDirectionField(st, curNode->m_point, nextNode->m_point, freq);
		}
		else if (curNode->m_type == NODE_DIFF) {
			CalculateDiffractionField_ReverseRT(efield, st, preNode->m_point, curNode->m_point, nextNode->m_point, curNode->m_wedge, matCurNode, freq, tranFunctionData);
		}
		else {
			LOG_ERROR << "Raypath: error in raypath information." << ENDL;
		}
	}
	return efield;				//直接返回三维复电场
}

Polarization3D CalculateStrengthField3DReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna)
{
	if (path->m_nodes.size() < 2)		//不满足电磁计算的条件
		return Polarization3D();
	Polarization3D efield;								/** @brief	初始三维极化电磁场	*/
	PathNode3D* curNode = nullptr;
	PathNode3D* preNode = nullptr;
	PathNode3D* nextNode = nullptr;
	preNode = path->m_nodes.back();
	curNode = std::prev(path->m_nodes.back());
	//首节点--计算辐射场
	Vector3D pc = curNode->m_point - preNode->m_point;												/** @brief	前一节点指向当前节点的向量	*/
	RtLbsType st = pc.Length();																		/** @brief	累积距离	*/
	CalRadiationField_ReverseRT(txAntenna, power, freq, pc.Azimuth(), pc.Elevation(), st, efield);		/** @brief	计算初始辐射场	*/
	//计算中间节点的辐射场
	for (auto it = std::next(path->m_nodes.rbegin()); it != std::prev(path->m_nodes.rend()); ++it) {
		if (it != std::next(path->m_nodes.rbegin()))										//当且仅当在首节点时不会进行前节点替换操作
			preNode = curNode;							//更新前一节点
		curNode = *it;							//更新当前节点
		nextNode = *(it + 1);					//更新下一节点
		Material* matCurNode = curNode->m_mat;
		if (curNode->m_type == NODE_REFL) {
			CalculateReflectionField_ReverseRT(efield, st, preNode->m_point, curNode->m_point, nextNode->m_point, matCurNode, freq);
		}
		else if (curNode->m_type == NODE_TRANIN || curNode->m_type == NODE_TRANOUT) {
			CalculateTransmissionField_ReverseRT(efield, st, preNode->m_point, curNode->m_point, nextNode->m_point, matCurNode, freq);
		}
		else if (curNode->m_type == NODE_ETRANIN) {
			CalculateStraightTransmissionField_ReverseRT(efield, st, curNode->m_point, nextNode->m_point, matCurNode, freq);
		}
		else if (curNode->m_type == NODE_ETRANOUT) {
			efield.CalculateDirectionField(st, curNode->m_point, nextNode->m_point, freq);
		}
		else if (curNode->m_type == NODE_DIFF) {
			CalculateDiffractionField_ReverseRT(efield, st, preNode->m_point, curNode->m_point, nextNode->m_point, curNode->m_wedge, matCurNode, freq, tranFunctionData);
		}
		else {
			LOG_ERROR << "Raypath: error in raypath information." << ENDL;
		}
	}
	return efield;				//直接返回三维复电场
}

Complex CalculateStrengthField(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	Polarization3D efield3d = CalculateStrengthField3D(path, power, freq, tranFunctionData, txAntenna);			/** @brief	计算三维复电磁场	*/
	Complex raypathEField;																							/** @brief	输出的复电场值	*/
	CalReceivedField(rxAntenna, efield3d, freq, path->GetAOA_Phi(), path->GetAOA_Theta(), raypathEField);
	return raypathEField;
}

Complex CalculateStrengthFieldReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	Polarization3D efield3d = CalculateStrengthField3DReverse(path, power, freq, tranFunctionData, txAntenna);			/** @brief	计算三维复电磁场	*/
	Complex raypathEField;																									/** @brief	输出的复电场值	*/
	CalReceivedField(rxAntenna, efield3d, freq, path->GetAOA_Phi(), path->GetAOA_Theta(), raypathEField);
	return raypathEField;
}

RtLbsType CalculatePowerInLBSSystem(const RayPath3D* path, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* trxAntenna)
{
	//先计算输出场
	RtLbsType power = 1.0;				/** @brief	默认功率设定为1W	*/
	Polarization3D efield3d = CalculateStrengthField3D(path, power, freq, tranFunctionData, trxAntenna);							/** @brief	计算三维复电磁场	*/
	Complex raypathEField;				/** @brief	路径场值	*/
	CalReceivedField(trxAntenna, efield3d, freq, path->GetAOA_Phi(), path->GetAOA_Theta(), raypathEField);
	//计算能量
	RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (freq / QUARTER_PI)) - 20 * log10(30);								/** @brief	电磁计算中间变量	*/
	RtLbsType receivedPower = 20 * log10(raypathEField.MValue()) + conterm + 30;											/** @brief	计算路径功率	*/
	return receivedPower;
}
