#include "pathnodegpu.h"

HOST_DEVICE_FUNC PathNodeGPU::PathNodeGPU()
	: m_isValid(false)
	, m_layer(-1)
	, m_rxId(-1)
	, m_ft(0.0)
{
}

PathNodeGPU::PathNodeGPU(const PathNodeGPU& node)
	: m_isValid(node.m_isValid)
	, m_layer(node.m_layer)
	, m_rxId(node.m_rxId)
	, m_inter(node.m_inter)
	, m_ft(node.m_ft)
{
}

HOST_DEVICE_FUNC PathNodeGPU::PathNodeGPU(bool isValid, int layer, int rxId, Intersection2DGPU inter)
	: m_isValid(isValid)
	, m_layer(layer)
	, m_rxId(rxId)
	, m_inter(inter)
	, m_ft(0.0)
{
}

HOST_DEVICE_FUNC PathNodeGPU& PathNodeGPU::operator=(const PathNodeGPU& node)
{
	m_isValid = node.m_isValid;
	m_layer = node.m_layer;
	m_rxId = node.m_rxId;
	m_inter = node.m_inter;
	m_ft = node.m_ft;
	return *this;
}


