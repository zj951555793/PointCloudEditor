#include "JMEngine/RulerMvsSlam.h"

#include <memory>
#include <string>

namespace JMEngine {
namespace {

class AndroidDisabledSlam final : public ISlam {
  ublic:
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
    return std::make_unique<AndroidDisabledSlam>();
}

bool rulerMvsAvailable() noexcept {
    return false;
}

} // namespace JMEngine
