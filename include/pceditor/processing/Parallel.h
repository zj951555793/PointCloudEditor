#pragma once
#include <algorithm>
#ifdef PCEDITOR_USE_OPENMP
#include <omp.h>
#endif
namespace pceditor::processing {
inline int processingThreadCount() noexcept {
#ifdef PCEDITOR_USE_OPENMP
    return std::max(1, omp_get_num_procs() - 1);
#else
    return 1;
#endif
}
} // namespace pceditor::processing
