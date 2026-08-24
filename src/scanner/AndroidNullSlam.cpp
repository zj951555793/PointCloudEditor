#include "JMEngine/RulerMvsSlam.h"

#include <memory>
#include <string>

namespace JMEngine {
namespace {

// Android camera-only build placeholder.
// It deliberately provides no SLAM implementation so the Android app can keep
// the shared JMScanner/Engine ABI without linking RulerMVS or OpenCV.
class AndroidNullSlam final : public ISlam {
  public:
    bool initialize(const ScanConfig&, std::string* error) override {
        if (error) {
            *error = "RulerMVS is disabled in the Android camera-only build";
        }
        return false;
    }

    bool process(const CameraFrame&) override {
        return false;
    }

    Pose pose() const override {
        return {};
    }

    std::shared_ptr<PointCloud> cloud() override {
        return {};
    }
};

} // namespace

std::unique_ptr<ISlam> createRulerMvsSlam() {
    return std::make_unique<AndroidNullSlam>();
}

} // namespace JMEngine
