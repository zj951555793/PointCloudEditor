#include "JMEngine/ScanProject.h"
#include "JMEngine/PointCloudIO.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace JMEngine {

namespace {
Vec3f transformProjectPoint(const Pose& pose, const Vec3f& p) {
    const auto& m = pose.matrix;
    return {
        m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
        m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
        m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
}

Vec3f transformProjectNormal(const Pose& pose, const Vec3f& n) {
    const auto& m = pose.matrix;
    Vec3f out{
        m[0] * n.x + m[4] * n.y + m[8] * n.z,
        m[1] * n.x + m[5] * n.y + m[9] * n.z,
        m[2] * n.x + m[6] * n.y + m[10] * n.z};
    const float length = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
    if (length > 1.0e-8f) {
        out.x /= length;
        out.y /= length;
        out.z /= length;
    }
    return out;
}

bool saveBinaryPlyWithNormals(const std::filesystem::path& file, const PointCloud& cloud) {
    std::ofstream out(file, std::ios::binary);
    if (!out)
        return false;

    out << "ply\nformat binary_little_endian 1.0\n";
    out << "element vertex " << cloud.size() << "\n";
    out << "property float x\nproperty float y\nproperty float z\n";
    out << "property float nx\nproperty float ny\nproperty float nz\n";
    out << "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\n";
    out << "end_header\n";

    auto writeFloat = [&out](float value) {
        out.write(reinterpret_cast<const char*>(&value), sizeof(float));
    };
    auto writeU8 = [&out](std::uint8_t value) {
        out.write(reinterpret_cast<const char*>(&value), sizeof(std::uint8_t));
    };

    for (const auto& p : cloud.points()) {
        writeFloat(p.position.x);
        writeFloat(p.position.y);
        writeFloat(p.position.z);
        writeFloat(p.normal.x);
        writeFloat(p.normal.y);
        writeFloat(p.normal.z);
        writeU8(static_cast<std::uint8_t>(p.rgba & 255u));
        writeU8(static_cast<std::uint8_t>((p.rgba >> 8u) & 255u));
        writeU8(static_cast<std::uint8_t>((p.rgba >> 16u) & 255u));
        writeU8(static_cast<std::uint8_t>((p.rgba >> 24u) & 255u));
    }
    return static_cast<bool>(out);
}
} // namespace

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
    if (ec) return false;

    const auto dir = root_ / "frames";
    if (std::filesystem::exists(dir)) {
        for (auto& e : std::filesystem::directory_iterator(dir)) {
            if (!e.is_directory()) continue;
            auto name = e.path().filename().string();
            if (name.rfind("frame_", 0) != 0) continue;
            int id = -1;
            try { id = std::stoi(name.substr(6)); } catch (...) { continue; }
            frames_.push_back({id, e.path() / "cloud.ply", e.path() / "pose.txt"});
        }
        std::sort(frames_.begin(), frames_.end(), [](const ScanProjectFrameInfo& a, const ScanProjectFrameInfo& b) {
            return a.frameId < b.frameId;
        });
    }

    if (!std::filesystem::exists(root_ / "project.json")) {
        std::ofstream project(root_ / "project.json");
        if (!project) return false;
        project << "{\n"
                << "  \"version\": 3,\n"
                << "  \"poseFile\": \"pose.txt\",\n"
                << "  \"textureStored\": false,\n"
                << "  \"frameCount\": " << frames_.size() << ",\n"
                << "  \"optimized\": false\n"
                << "}\n";
    }
    return true;
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

    if (!saveBinaryPlyWithNormals(frameDir(frameId) / "cloud.ply", cloud))
        return false;
    const bool poseOk = savePose(frameId, pose);
    if (poseOk) {
        const auto existing = std::find_if(frames_.begin(), frames_.end(), [frameId](const ScanProjectFrameInfo& f) {
            return f.frameId == frameId;
        });
        if (existing == frames_.end())
            frames_.push_back({frameId, frameDir(frameId) / "cloud.ply", frameDir(frameId) / "pose.txt"});
        else
            *existing = {frameId, frameDir(frameId) / "cloud.ply", frameDir(frameId) / "pose.txt"};

        std::ofstream json(root_ / "project.json");
        if (json) {
            json << "{\n";
            json << "  \"version\": 3,\n";
            json << "  \"poseFile\": \"pose.txt\",\n";
            json << "  \"textureStored\": false,\n";
            json << "  \"frameCount\": " << frames_.size() << ",\n";
            json << "  \"optimized\": false\n";
            json << "}\n";
        }
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
    std::string error;
    auto loaded = PointCloudIO::loadPly(file.string(), &error);
    if (!loaded)
        return false;
    cloud = std::move(*loaded);
    return true;
}

bool ScanProject::loadFrameCloud(int frameId, PointCloud& cloud) const
{
    return loadPly(frameDir(frameId) / "cloud.ply", cloud);
}

bool ScanProject::saveMergedPly(const std::string& file, const PointCloud& cloud) const
{
    return saveBinaryPlyWithNormals(file, cloud);
}

bool ScanProject::rebuildProjectCloud(PointCloud& merged) const
{
    merged.points().clear();
    for (const auto& f : frames_) {
        PointCloud cloud;
        Pose pose;
        if (!loadPly(f.cloudFile, cloud) || !loadFramePose(f.frameId, pose)) continue;
        auto& dst = merged.points();
        dst.reserve(dst.size() + cloud.points().size());
        for (auto point : cloud.points()) {
            point.position = transformProjectPoint(pose, point.position);
            point.normal = transformProjectNormal(pose, point.normal);
            dst.push_back(point);
        }
    }
    return !merged.empty();
}

bool ScanProject::rebuildProjectCloud(const std::string& outputFile) const
{
    PointCloud merged;
    if (!rebuildProjectCloud(merged))
        return false;
    return saveMergedPly(outputFile, merged);
}

}
