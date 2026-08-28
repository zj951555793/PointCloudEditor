#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace nanoflann {

struct KDTreeSingleIndexAdaptorParams {
    explicit KDTreeSingleIndexAdaptorParams(int leafMaxSize = 10) : leaf_max_size(leafMaxSize) {}
    int leaf_max_size;
};

struct SearchParameters {
    explicit SearchParameters(float epsValue = 0.0f) : eps(epsValue) {}
    float eps;
};

template <typename T, typename DataSource, typename DistanceType = T>
struct L2_Simple_Adaptor {
    using ElementType = T;
    using ResultType = DistanceType;

    explicit L2_Simple_Adaptor(const DataSource& dataSource) : data_source(dataSource) {}

    const DataSource& data_source;
};

template <typename DistanceType, typename IndexType = std::size_t, typename CountType = std::size_t>
class KNNResultSet {
  public:
    explicit KNNResultSet(CountType capacity) : capacity_(capacity) {}

    void init(IndexType* indices, DistanceType* distances) {
        indices_ = indices;
        distances_ = distances;
        count_ = 0;
    }

    bool addPoint(DistanceType distance, IndexType index) {
        if (!indices_ || !distances_ || capacity_ == 0) return true;
        CountType pos = count_;
        if (count_ < capacity_) {
            ++count_;
        } else if (distance >= distances_[capacity_ - 1]) {
            return true;
        } else {
            pos = capacity_ - 1;
        }

        while (pos > 0 && distances_[pos - 1] > distance) {
            if (pos < capacity_) {
                distances_[pos] = distances_[pos - 1];
                indices_[pos] = indices_[pos - 1];
            }
            --pos;
        }
        distances_[pos] = distance;
        indices_[pos] = index;
        return true;
    }

    CountType size() const { return count_; }
    bool full() const { return count_ == capacity_; }
    DistanceType worstDist() const {
        return full() ? distances_[capacity_ - 1] : std::numeric_limits<DistanceType>::max();
    }

  private:
    CountType capacity_{0};
    CountType count_{0};
    IndexType* indices_{nullptr};
    DistanceType* distances_{nullptr};
};

template <typename Distance, typename DatasetAdaptor, int DIM = -1, typename IndexType = std::size_t>
class KDTreeSingleIndexAdaptor {
  public:
    using ElementType = typename Distance::ElementType;
    using DistanceType = typename Distance::ResultType;

    KDTreeSingleIndexAdaptor(int dimensionality, const DatasetAdaptor& inputData,
                             const KDTreeSingleIndexAdaptorParams& params =
                                 KDTreeSingleIndexAdaptorParams())
        : dim_(dimensionality), data_(inputData), params_(params) {}

    void buildIndex() {}

    template <typename ResultSet>
    bool findNeighbors(ResultSet& result, const ElementType* query,
                       const SearchParameters& = SearchParameters()) const {
        const std::size_t count = data_.kdtree_get_point_count();
        for (std::size_t index = 0; index < count; ++index) {
            DistanceType distance = DistanceType();
            for (int dim = 0; dim < dim_; ++dim) {
                const DistanceType delta =
                    static_cast<DistanceType>(query[dim] - data_.kdtree_get_pt(index, dim));
                distance += delta * delta;
            }
            result.addPoint(distance, static_cast<IndexType>(index));
        }
        return true;
    }

  private:
    int dim_{0};
    const DatasetAdaptor& data_;
    KDTreeSingleIndexAdaptorParams params_;
};

} // namespace nanoflann
