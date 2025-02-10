#include "raypath.h"
#include "core/render/pathtracing/pathtracing.h"
#include "scene/scene.h"
#include "radiowave/raypath/gpu/cpuconverterpathnode.h"

RayPath::RayPath()
    : m_bContainRefract(false)
    , m_angularSpectrumCategoryId(-1)
{
}

RayPath::RayPath(const std::vector<PathNode*>& nodes, bool containRefract)
    : m_bContainRefract(containRefract)
    , m_angularSpectrumCategoryId(-1)
{
	for (auto& curNode : nodes) {
		m_nodes.push_back(new PathNode(*curNode));
	}
}

RayPath::RayPath(const RayPath& path)
	: m_bContainRefract(path.m_bContainRefract)
	, m_angularSpectrumCategoryId(path.m_angularSpectrumCategoryId)
{
	for (auto& curNode : path.m_nodes) {
		m_nodes.push_back(new PathNode(*curNode));
	}
}


RayPath::~RayPath()
{
	for (auto& curNode : m_nodes) {
		if (curNode != nullptr) {
			delete curNode;
			curNode = nullptr;
		}
	}
}

bool RayPath::operator==(const RayPath& path) const
{
	const std::vector<PathNode*>& nodesA = m_nodes;
	const std::vector<PathNode*>& nodesB = path.m_nodes;
	if (nodesA.size() != nodesB.size())
		return false;
	for (size_t i = 0; i < nodesA.size(); ++i) {
		if (nodesA[i]->m_type != nodesB[i]->m_type)
			return false;
		if (nodesA[i]->m_type == NODE_REFL || 
            nodesA[i]->m_type == NODE_TRANIN ||
            nodesA[i]->m_type == NODE_TRANOUT ||
            nodesA[i]->m_type == NODE_ETRANIN ||
            nodesA[i]->m_type == NODE_ETRANOUT) {
			if (nodesA[i]->m_segment != nodesB[i]->m_segment)
				return false;
		}
		if (nodesA[i]->m_type == NODE_DIFF) {
			if (nodesA[i]->m_wedge != nodesB[i]->m_wedge)
				return false;
		}
	}
	return true;
}

bool RayPath::operator!=(const RayPath& path) const
{
    return !(*this == path);
}

void RayPath::DeepCopy(const RayPath* path)
{
    if (path == nullptr)
        return;
    m_bContainRefract = path->m_bContainRefract;
    m_angularSpectrumCategoryId = path->m_angularSpectrumCategoryId;
    m_nodes.resize(path->m_nodes.size());
    for (int i = 0; i < path->m_nodes.size(); ++i) {
        m_nodes[i] = new PathNode(*path->m_nodes[i]);
    }
}

void RayPath::DeepDestroy()
{
    for (int i = 0; i < m_nodes.size(); ++i) {
        delete m_nodes[i];
    }
}

void RayPath::Clear()
{
	m_nodes.clear();
	m_bContainRefract = false;
}

void RayPath::Write2File(std::ofstream& stream) const
{
    stream << m_nodes.size() << ",";
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        stream << m_nodes[i]->m_point.x << "," << m_nodes[i]->m_point.y << ",";
    }
    stream << "\n";
}

void RayPath::ConvertFrom(const std::vector<CPUConverterPathNode*>& nodes, const std::vector<Segment2D*>& segments, const std::vector<Wedge2D*>& wedges)
{
	m_nodes.resize(nodes.size());
	for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        PathNode* newNode = new PathNode();
        newNode->ConvertFrom(*nodes[i], segments, wedges);
        m_nodes[i] = newNode;
        if (newNode->m_type == NODE_TRANOUT) {
            m_bContainRefract = true;
        }
	}
	//调整末尾节点广义源位置
	m_nodes[m_nodes.size() - 1]->m_source = m_nodes[m_nodes.size() - 2]->m_source;
}
