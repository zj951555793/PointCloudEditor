#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace JMEngine {

struct ProjectInfo {
    std::string name;
    std::filesystem::path path;
    std::filesystem::path previewImage;
    int frameCount{0};
    bool optimized{false};
};

class ProjectManager {
public:
    static std::filesystem::path defaultWorkspace();

    bool createProject(const std::filesystem::path& path);
    bool openProject(const std::filesystem::path& path, ProjectInfo& info);
    std::vector<ProjectInfo> scanProjects(const std::filesystem::path& workspace) const;

    bool savePreview(const std::filesystem::path& project,
                     const std::string& imageFile);

private:
    bool readProjectJson(const std::filesystem::path& file, ProjectInfo& info) const;
};

}
