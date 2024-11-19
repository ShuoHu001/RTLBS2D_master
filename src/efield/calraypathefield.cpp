#include "calraypathefield.h"

Polarization3D CalculateStrengthField3D(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna)
{
	if (path->m_nodes.size() < 2)		//�������ż��������
		return Polarization3D();
	Polarization3D efield;								/** @brief	��ʼ��ά������ų�	*/
	PathNode3D* curNode = nullptr;
	PathNode3D* preNode = nullptr;
	PathNode3D* nextNode = nullptr;
	preNode = path->m_nodes[0];
	curNode = path->m_nodes[1];
	//�׽ڵ�--������䳡
	Vector3D pc = curNode->m_point - preNode->m_point;												/** @brief	ǰһ�ڵ�ָ��ǰ�ڵ������	*/
	RtLbsType st = pc.Length();																		/** @brief	�ۻ�����	*/
	CalRadiationField_ReverseRT(txAntenna, power, freq, pc.Azimuth(), pc.Elevation(), st, efield);		/** @brief	�����ʼ���䳡	*/
	//�����м�ڵ�ķ��䳡
	for (int i = 1; i < path->m_nodes.size() - 1; ++i) {
		if (i != 1)										//���ҽ������׽ڵ�ʱ�������ǰ�ڵ��滻����
			preNode = curNode;							//����ǰһ�ڵ�
		curNode = path->m_nodes[i];						//���µ�ǰ�ڵ�
		nextNode = path->m_nodes[i + 1];					//������һ�ڵ�
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
	return efield;				//ֱ�ӷ�����ά���糡
}

Polarization3D CalculateStrengthField3DReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna)
{
	if (path->m_nodes.size() < 2)		//�������ż��������
		return Polarization3D();
	Polarization3D efield;								/** @brief	��ʼ��ά������ų�	*/
	PathNode3D* curNode = nullptr;
	PathNode3D* preNode = nullptr;
	PathNode3D* nextNode = nullptr;
	preNode = path->m_nodes.back();
	curNode = std::prev(path->m_nodes.back());
	//�׽ڵ�--������䳡
	Vector3D pc = curNode->m_point - preNode->m_point;												/** @brief	ǰһ�ڵ�ָ��ǰ�ڵ������	*/
	RtLbsType st = pc.Length();																		/** @brief	�ۻ�����	*/
	CalRadiationField_ReverseRT(txAntenna, power, freq, pc.Azimuth(), pc.Elevation(), st, efield);		/** @brief	�����ʼ���䳡	*/
	//�����м�ڵ�ķ��䳡
	for (auto it = std::next(path->m_nodes.rbegin()); it != std::prev(path->m_nodes.rend()); ++it) {
		if (it != std::next(path->m_nodes.rbegin()))										//���ҽ������׽ڵ�ʱ�������ǰ�ڵ��滻����
			preNode = curNode;							//����ǰһ�ڵ�
		curNode = *it;							//���µ�ǰ�ڵ�
		nextNode = *(it + 1);					//������һ�ڵ�
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
	return efield;				//ֱ�ӷ�����ά���糡
}

Complex CalculateStrengthField(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	Polarization3D efield3d = CalculateStrengthField3D(path, power, freq, tranFunctionData, txAntenna);			/** @brief	������ά����ų�	*/
	Complex raypathEField;																							/** @brief	����ĸ��糡ֵ	*/
	CalReceivedField(rxAntenna, efield3d, freq, path->GetAOA_Phi(), path->GetAOA_Theta(), raypathEField);
	return raypathEField;
}

Complex CalculateStrengthFieldReverse(const RayPath3D* path, RtLbsType power, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* txAntenna, const Antenna* rxAntenna)
{
	Polarization3D efield3d = CalculateStrengthField3DReverse(path, power, freq, tranFunctionData, txAntenna);			/** @brief	������ά����ų�	*/
	Complex raypathEField;																									/** @brief	����ĸ��糡ֵ	*/
	CalReceivedField(rxAntenna, efield3d, freq, path->GetAOA_Phi(), path->GetAOA_Theta(), raypathEField);
	return raypathEField;
}

RtLbsType CalculatePowerInLBSSystem(const RayPath3D* path, RtLbsType freq, const std::vector<Complex>& tranFunctionData, const Antenna* trxAntenna)
{
	//�ȼ��������
	RtLbsType power = 1.0;				/** @brief	Ĭ�Ϲ����趨Ϊ1W	*/
	Polarization3D efield3d = CalculateStrengthField3D(path, power, freq, tranFunctionData, trxAntenna);							/** @brief	������ά����ų�	*/
	Complex raypathEField;				/** @brief	·����ֵ	*/
	CalReceivedField(trxAntenna, efield3d, freq, path->GetAOA_Phi(), path->GetAOA_Theta(), raypathEField);
	//��������
	RtLbsType conterm = 20 * log10(LIGHT_VELOCITY_AIR / (freq / QUARTER_PI)) - 20 * log10(30);								/** @brief	��ż����м����	*/
	RtLbsType receivedPower = 20 * log10(raypathEField.MValue()) + conterm + 30;											/** @brief	����·������	*/
	return receivedPower;
}
