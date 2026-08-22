#include "JMEngine/ScanProject.h"
#include <fstream>
#include <iomanip>
#include <sstream>

namespace JMEngine {

std::filesystem::path ScanProject::frameDir(int frameId) const {
    return root_ / "frames" / ("frame_" + std::to_string(frameId));
}

bool ScanProject::open(const std::string& path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    root_ = path;
    frames_.clear();
    std::error_code ec;
    std::filesystem::create_directories(root_ / "frames", ec);
    std::ofstream project(root_ / "project.json");
    if (project) {
        project << "{\n  \"version\": 2,\n  \"poseFile\": \"pose.txt\",\n  \"textureStored\": false\n}\n";
    }
    return !ec;
}

bool ScanProject::openExisting(const std::string& path)
{
    std::lock_guard<std::mutex> lock(mutex_);
    root_ = path;
    frames_.clear();
    auto dir = root_ / "frames";
    if (!std::filesystem::exists(dir)) return false;
    for (auto& e : std::filesystem::directory_iterator(dir)) {
        if (!e.is_directory()) continue;
        auto name = e.path().filename().string();
        if (name.rfind("frame_", 0) != 0) continue;
        int id = -1;
        try { id = std::stoi(name.substr(6)); } catch (...) { continue; }
        frames_.push_back({id, e.path() / "cloud.ply", e.path() / "pose.txt"});
    }
    return true;
}

void ScanProject::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    root_.clear();
    frames_.clear();
}

bool ScanProject::savePose(int frameId, const Pose& pose)
{
    std::ofstream out(frameDir(frameId) / "pose.txt");
    if (!out) return false;
    out << std::setprecision(9);
    for (float v : pose.matrix) out << v << ' ';
    return true;
}

bool ScanProject::saveFrame(int frameId, const Pose& pose, const PointCloud& cloud)
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::error_code ec;
    std::filesystem::create_directories(frameDir(frameId), ec);
    if (ec) return false;

    std::ofstream ply(frameDir(frameId) / "cloud.ply");
    if (!ply) return false;
    ply << "ply\nformat ascii 1.0\n";
    ply << "element vertex " << cloud.size() << "\n";
    ply << "property float x\nproperty float y\nproperty float z\n";
    ply << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for (const auto& p : cloud.points()) {
        unsigned int c = p.rgba;
        ply << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
            << ((c >> 16) & 255) << ' ' << ((c >> 8) & 255) << ' ' << (c & 255) << '\n';
    }
    const bool poseOk = savePose(frameId, pose);
    if (poseOk) {
        std::ofstream json(root_ / "project.json");
        if (json) {
            json << "{\n";
            json << "  \"version\": 3,\n";
            json << "  \"poseFile\": \"pose.txt\",\n";
            json << "  \"textureStored\": false,\n";
            json << "  \"frameCount\": " << (frames_.size() + 1) << "\n";
            json << "}\n";
        }
        frames_.push_back({frameId, frameDir(frameId) / "cloud.ply", frameDir(frameId) / "pose.txt"});
    }
    return poseOk;
}

std::vector<ScanProjectFrameInfo> ScanProject::frames() const { return frames_; }

bool ScanProject::loadFramePose(int frameId, Pose& pose) const
{
    std::ifstream in(frameDir(frameId) / "pose.txt");
    if (!in) return false;
    for (float& v : pose.matrix) in >> v;
    return true;
}

bool ScanProject::loadPly(const std::filesystem::path& file, PointCloud& cloud) const
{
    std::ifstream in(file);
    if (!in) return false;
    std::string line;
    size_t count = 0;
    while (std::getline(in, line)) {
        if (line.rfind("element vertex", 0) == 0) count = std::stoul(line.substr(15));
        if (line == "end_header") break;
    }
    cloud.points().clear();
    cloud.points().reserve(count);
    for (size_t i = 0; i < count; ++i) {
        Point p; int r,g,b;
        if (!(in >> p.position.x >> p.position.y >> p.position.z >> r >> g >> b)) break;
        p.rgba = (uint32_t(r)<<16)|(uint32_t(g)<<8)|uint32_t(b);
        cloud.points().push_back(p);
    }
    return true;
}

bool ScanProject::loadFrameCloud(int frameId, PointCloud& cloud) const
{
    return loadPly(frameDir(frameId) / "cloud.ply", cloud);
}

bool ScanProject::saveMergedPly(const std::string& file, const PointCloud& cloud) const
{
    std::ofstream out(file);
    if (!out) return false;
    out << "ply\nformat ascii 1.0\n";
    out << "element vertex " << cloud.size() << "\nproperty float x\nproperty float y\nproperty float z\n";
    out << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for (auto& p: cloud.points()) {
        unsigned c=p.rgba;
        out<<p.position.x<<' '<<p.position.y<<' '<<p.position.z<<' '
           <<((c>>16)&255)<<' '<<((c>>8)&255)<<' '<<(c&255)<<'\n';
    }
    return true;
}

bool ScanProject::rebuildProjectCloud(const std::string& outputFile) const
{
    PointCloud merged;
    for (const auto& f : frames_) {
        PointCloud cloud;
        Pose pose;
        if (!loadPly(f.cloudFile, cloud) || !loadFramePose(f.frameId, pose)) continue;
        // TODO: apply full transform using project Vec3 math in reconstruction module.
        // Keep raw merge path here; optimizer can replace pose.txt before rebuild.
        auto& dst = merged.points();
        dst.insert(dst.end(), cloud.points().begin(), cloud.points().end());
    }
    return saveMergedPly(outputFile, merged);
}

}
