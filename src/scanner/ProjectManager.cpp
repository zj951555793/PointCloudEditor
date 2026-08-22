#include "JMEngine/ProjectManager.h"
#include <fstream>
#include <cstdlib>

namespace JMEngine {

std::filesystem::path ProjectManager::defaultWorkspace()
{
#ifdef _WIN32
    return "D:/workspace";
#else
    const char* home = std::getenv("HOME");
    if (home) return std::filesystem::path(home) / "workspace";
    return "./workspace";
#endif
}

bool ProjectManager::createProject(const std::filesystem::path& path)
{
    std::error_code ec;
    std::filesystem::create_directories(path / "frames", ec);
    std::filesystem::create_directories(path / "result", ec);
    if (ec) return false;

    std::ofstream out(path / "project.json");
    if (!out) return false;
    out << "{\n"
        << "  \"version\":3,\n"
        << "  \"textureStored\":false,\n"
        << "  \"frameCount\":0,\n"
        << "  \"optimized\":false\n"
        << "}\n";
    return true;
}

bool ProjectManager::readProjectJson(const std::filesystem::path& file,
                                     ProjectInfo& info) const
{
    std::ifstream in(file);
    if (!in) return false;
    info.name = file.parent_path().filename().string();
    info.path = file.parent_path();
    info.previewImage = info.path / "preview.png";
    std::string s((std::istreambuf_iterator<char>(in)), {});
    auto p=s.find("\"frameCount\":");
    if(p!=std::string::npos) info.frameCount=std::atoi(s.c_str()+p+14);
    p = s.find("\"optimized\"");
    if (p != std::string::npos) {
        const auto end = s.find_first_of(",}", p);
        const auto value = s.substr(p, end == std::string::npos ? std::string::npos : end - p);
        info.optimized = value.find("true") != std::string::npos;
    }
    return true;
}

bool ProjectManager::openProject(const std::filesystem::path& path,
                                 ProjectInfo& info)
{
    return readProjectJson(path / "project.json", info);
}

std::vector<ProjectInfo> ProjectManager::scanProjects(
        const std::filesystem::path& workspace) const
{
    std::vector<ProjectInfo> result;
    if (!std::filesystem::exists(workspace)) return result;
    for (auto& e : std::filesystem::directory_iterator(workspace)) {
        if (!e.is_directory()) continue;
        ProjectInfo info;
        if (readProjectJson(e.path()/"project.json", info))
            result.push_back(info);
    }
    return result;
}

bool ProjectManager::savePreview(const std::filesystem::path& project,
                                 const std::string& imageFile)
{
    std::error_code ec;
    std::filesystem::copy_file(imageFile,
                               project/"preview.png",
                               std::filesystem::copy_options::overwrite_existing,
                               ec);
    return !ec;
}

}
