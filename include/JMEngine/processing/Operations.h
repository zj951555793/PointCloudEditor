#pragma once
#include <JMEngine/processing/Processing.h>
namespace JMEngine::processing {
#define JMENGINE_DECLARE_OP(name)                                                                                      \
    class name final : public IProcessingOperation {                                                                   \
      public:                                                                                                          \
        OperationDescriptor descriptor() const override;                                                               \
        ProcessResult run(const ProcessInput&, const ParameterMap&, const ProgressCallback&,                           \
                          const CancelToken&) const override;                                                          \
    };
JMENGINE_DECLARE_OP(VoxelDownsampleOperation)
JMENGINE_DECLARE_OP(RadiusOutlierOperation)
JMENGINE_DECLARE_OP(StatisticalOutlierOperation)
JMENGINE_DECLARE_OP(SmallClusterOperation)
JMENGINE_DECLARE_OP(NormalEstimationOperation)
JMENGINE_DECLARE_OP(MeshCleanupOperation)
JMENGINE_DECLARE_OP(MeshDenoiseOperation)
JMENGINE_DECLARE_OP(LaplacianSmoothOperation)
JMENGINE_DECLARE_OP(TaubinSmoothOperation)
JMENGINE_DECLARE_OP(QemDecimateOperation)
JMENGINE_DECLARE_OP(HoleFillOperation)
JMENGINE_DECLARE_OP(OctreePoissonOperation)
#undef JMENGINE_DECLARE_OP
} // namespace JMEngine::processing
