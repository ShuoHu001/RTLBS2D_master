#include "materiallibraryconfig.h"



MaterialLibraryConfig::MaterialLibraryConfig()
{
}

MaterialLibraryConfig::~MaterialLibraryConfig()
{
}

bool MaterialLibraryConfig::Init(std::string filename)
{
    std::ifstream config_ifs(filename);
    if (!config_ifs.is_open()) {
        LOG_ERROR<<"MaterialLibraryConfig: fail to open "<< filename << ENDL;
        this->Write2Json(filename);
        LOG_INFO << "MaterialLibraryConfig: have wrote to default configuration to file: " << filename << ENDL;
        return false;
    }
    std::stringstream ss;
    ss << config_ifs.rdbuf();
    config_ifs.close();
    std::string jsonString = ss.str();
    rapidjson::Document doc;
    doc.Parse(jsonString.c_str());
    if (doc.HasMember(KEY_MATERIALLIBRARYCONFIG.c_str())) {
        rapidjson::Value& value = doc[KEY_MATERIALLIBRARYCONFIG.c_str()];
        if (value.IsObject()) {
            if (this->Deserialize(value)) {
                LOG_INFO << "MaterialLibraryConfig: load data success!" << ENDL;
                return true;
            }
        }
    }
    return false;
}

void MaterialLibraryConfig::Write2Json(std::string filename)
{
    std::ofstream config_ofs(filename);
    if (!config_ofs.is_open()) {
        LOG_ERROR << "MaterialLibraryConfig: fail to open " << filename << ENDL;
        return;
    }
    rapidjson::StringBuffer sb;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);
	writer.SetIndent('\t', 1);
	writer.StartObject();
    writer.Key(KEY_MATERIALLIBRARYCONFIG.c_str()); this->Serialize(writer);
    writer.EndObject();
    config_ofs << sb.GetString();
    config_ofs.close();
    return;
}

void MaterialLibraryConfig::Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer)
{
    writer.StartObject();
    writer.Key(KEY_MATERIALLIBRARYCONFIG_MATERIALS.c_str());                                        SerializeArray(m_materials, writer);
    writer.EndObject();
    return;
}

bool MaterialLibraryConfig::Deserialize(const rapidjson::Value& value)
{
    if (!value.IsObject()) {
		LOG_ERROR << "MaterialLibraryConfig: not a json object." << ENDL;
		return false;
    }
    
    if (!value.HasMember(KEY_MATERIALLIBRARYCONFIG_MATERIALS.c_str())) {
		LOG_ERROR << "MaterialLibraryConfig: missing " << KEY_MATERIALLIBRARYCONFIG_MATERIALS.c_str() << ENDL;
		return false;
    }

    const rapidjson::Value& materialsValue = value[KEY_MATERIALLIBRARYCONFIG_MATERIALS.c_str()];

    if (!DeserializeArray(m_materials, materialsValue)) {
		LOG_ERROR << "MaterialLibraryConfig: " << KEY_MATERIALLIBRARYCONFIG_MATERIALS.c_str() << ", deserialize failed." << ENDL;
		return false;
	}

    return true;
}
