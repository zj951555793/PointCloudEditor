#pragma once

#include "PointCloud.h"
#include "ScanTypes.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace JMEngine {

class ISlam {
  public:
    using UpdateCallback = std::function<void(
        int, const Pose&, std::shared_ptr<PointCloud>,
        std::shared_ptr<PointCloud>, bool)>;
    using MarkerCallback = std::function<void(const ScanMarkerFrame&)>;

    virtual ~ISlam() = default;

    virtual bool initialize(const ScanConfig&, std::string*) {
        return true;
    }
    virtual bool process(const CameraFrame&) = 0;
    virtual Pose pose() const = 0;
    virtual std::shared_ptr<PointCloud> cloud() = 0;
    virtual std::shared_ptr<PointCloud> reconstruct(
        const std::function<void(int)>& progress, std::string*) {
        if (progress)
            progress(100);
        return cloud();
    }
    virtual void reset() {}
    virtual void setUpdateCallback(UpdateCallback) {}
    virtual void setMarkerCallback(MarkerCallback) {}
    virtual std::vector<TextureKeyframe> takeTextureKeyframes() {
        return {};
    }
};

} // namespace JMEngine
