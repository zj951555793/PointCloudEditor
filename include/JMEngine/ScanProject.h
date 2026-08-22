#pragma once

#include "PointCloud.h"
#include "ScanTypes.h"
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace JMEngine {

struct ScanProjectFrameInfo {
    int frameId{-1};
    std::filesystem::path cloudFile;
    std::filesystem::path poseFile;
};

// Optional provider for texture processing. ScanProject does not store images.
class IFrameImageProvider {
public:
    virtual ~IFrameImageProvider() = default;
    virtual bool getFrameImage(int frameId, std::vector<uint8_t>& rgb,
                               int& width, int& height) = 0;
};

class ScanProject {
public:
    bool open(const std::string& path);
    bool openExisting(const std::string& path);
    void close();

    bool saveFrame(int frameId, const Pose& pose, const PointCloud& cloud);
    bool savePose(int frameId, const Pose& pose);

    std::vector<ScanProjectFrameInfo> frames() const;
    bool loadFrameCloud(int frameId, PointCloud& cloud) const;
    bool loadFramePose(int frameId, Pose& pose) const;
    bool rebuildProjectCloud(PointCloud& cloud) const;
    bool rebuildProjectCloud(const std::string& outputFile) const;

private:
    std::filesystem::path frameDir(int frameId) const;
    bool loadPly(const std::filesystem::path& file, PointCloud& cloud) const;
    bool saveMergedPly(const std::string& file, const PointCloud& cloud) const;

private:
    std::filesystem::path root_;
    mutable std::mutex mutex_;
    std::vector<ScanProjectFrameInfo> frames_;
};

}
