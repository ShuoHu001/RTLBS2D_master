#include "materiallibrary.h"

MaterialLibrary::MaterialLibrary()
{
}

MaterialLibrary::MaterialLibrary(const MaterialLibraryConfig& config)
{
	const std::vector<Material>& materials = config.m_materials;
	m_materials.resize(materials.size());
	unsigned id = 0;
	for (auto it = materials.begin(); it != materials.end(); ++it) {
		Material* material = new Material(*it);
		m_materials[id++] = material;
	}
}

MaterialLibrary::~MaterialLibrary()
{
	for (auto& mat:m_materials) {
		delete mat;
		mat = nullptr;
	}
	m_materials.clear();
	std::vector<Material*>().swap(m_materials);
}

bool MaterialLibrary::Init(const MaterialLibraryConfig& config)
{
	const std::vector<Material>& materials = config.m_materials;
	if (materials.size() == 0)
		return false;
	m_materials.resize(materials.size());
	unsigned id = 0;
	for (auto it = materials.begin(); it != materials.end(); ++it) {
		Material* material = new Material(*it);
		m_materials[id++] = material;
	}
	return true;
}

Material* MaterialLibrary::GetMaterial(const std::string& matName) const
{
	for (auto it = m_materials.begin(); it != m_materials.end(); ++it) {
		Material* material = *it;
		if (matName == material->m_name) {
			return material;
		}
	}
	return GetDefaultMaterial();
}

int MaterialLibrary::GetMaterialId(const std::string& matName) const
{
	for (auto it = m_materials.begin(); it != m_materials.end(); ++it) {
		Material* material = *it;
		if (matName == material->m_name) {
			return material->m_id;
		}
	}
	return GetDefaultMaterial()->m_id;
}

Material* MaterialLibrary::GetMaterial(unsigned matId) const
{
	if (matId >= m_materials.size())
		return GetDefaultMaterial();
	return m_materials[matId];
}

Material* MaterialLibrary::GetDefaultMaterial() const
{
	return m_materials.front();
}
