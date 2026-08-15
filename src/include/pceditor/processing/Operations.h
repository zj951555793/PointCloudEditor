#pragma once
#include <pceditor/processing/Processing.h>
namespace pceditor::processing {
#define PCEDITOR_DECLARE_OP(name)                                                                                      \
    class name final : public IProcessingOperation {                                                                   \
      public:                                                                                                          \
        OperationDescriptor descriptor() const override;                                                               \
        ProcessResult run(const ProcessInput&, const ParameterMap&, const ProgressCallback&,                           \
                          const CancelToken&) const override;                                                          \
    };
PCEDITOR_DECLARE_OP(VoxelDownsampleOperation)
PCEDITOR_DECLARE_OP(RadiusOutlierOperation)
PCEDITOR_DECLARE_OP(StatisticalOutlierOperation)
PCEDITOR_DECLARE_OP(SmallClusterOperation)
PCEDITOR_DECLARE_OP(NormalEstimationOperation)
PCEDITOR_DECLARE_OP(MeshCleanupOperation)
PCEDITOR_DECLARE_OP(LaplacianSmoothOperation)
PCEDITOR_DECLARE_OP(TaubinSmoothOperation)
PCEDITOR_DECLARE_OP(QemDecimateOperation)
PCEDITOR_DECLARE_OP(HoleFillOperation)
PCEDITOR_DECLARE_OP(OctreePoissonOperation)
#undef PCEDITOR_DECLARE_OP
} // namespace pceditor::processing
