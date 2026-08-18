#pragma once

#include "ISlam.h"

#include <memory>

namespace JMEngine {

std::unique_ptr<ISlam> createRulerMvsSlam();
bool rulerMvsAvailable() noexcept;

} // namespace JMEngine
