#include "matmanager.h"

Material* MatManager::FindMaterial(const std::string& mat_name) const
{
    std::map<std::string, Material*>::const_iterator it = m_matPool.find(mat_name);
    if (it != m_matPool.end())
        return it->second;
    return 0;
}

Material* MatManager::GetDefaultMat() const
{
    return m_Default;
}
