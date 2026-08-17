#pragma once
#include <algorithm>
#ifdef JMENGINE_USE_OPENMP
#include <omp.h>
#endif
namespace JMEngine::processing {
inline int processingThreadCount() noexcept {
#ifdef JMENGINE_USE_OPENMP
    return std::max(1, omp_get_num_procs() - 1);
#else
    return 1;
#endif
}
} // namespace JMEngine::processing
