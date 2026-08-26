#pragma once

#include <JMEngine/PointCloud.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_set>
#include <vector>

namespace JMEngine::processing::detail {

struct CloudScaleEstimate {
    double diagonal{1.0};               // cloud/model units
    double spacing{0.001};              // cloud/model units
    double unitsPerMillimeter{0.001};   // 1 mm expressed in cloud/model units
    std::size_t estimatedActivePoints{0};
};

inline double intervalPenalty(double value, double lo, double hi) {
    if (!(value > 0.0) || !std::isfinite(value))
        return 100.0;
    if (value < lo)
        return std::log(lo / value);
    if (value > hi)
        return std::log(value / hi);
    return 0.0;
}

// Scanner projects historically contain both metre and millimetre coordinates.  A single
// bbox threshold (the old "diagonal >= 50 => mm") is ambiguous for small objects: a 30 mm
// scan was interpreted as 30 metres.  Infer the unit using BOTH physical object size and
// sampling pitch.  The candidate whose converted values look most like a structured-light
// scan wins.  This keeps 0.3m/1mm data in metres while correctly recognizing 30mm/0.2mm data.
inline double inferUnitsPerMillimeter(double diagonalUnits, double spacingUnits) {
    auto score = [&](double unitToMm) {
        const double diagonalMm = diagonalUnits * unitToMm;
        const double spacingMm = spacingUnits * unitToMm;
        double s = 0.0;
        // Broad production ranges; penalties are logarithmic rather than hard cutoffs.
        s += 2.0 * intervalPenalty(diagonalMm, 5.0, 5000.0);
        s += 2.5 * intervalPenalty(spacingMm, 0.01, 10.0);
        const double ratio = spacingMm / std::max(1e-12, diagonalMm);
        s += intervalPenalty(ratio, 1e-6, 0.08);
        return s;
    };

    const double mmScore = score(1.0);       // one cloud unit = one mm
    const double metreScore = score(1000.0); // one cloud unit = one metre
    return mmScore <= metreScore ? 1.0 : 0.001;
}

inline CloudScaleEstimate estimateCloudScale(const PointCloud& cloud) {
    CloudScaleEstimate out;
    if (cloud.empty())
        return out;

    constexpr std::size_t kMaxSamples = 2048;
    const auto& pts = cloud.points();

    std::vector<Vec3f> sample;
    sample.reserve(std::min<std::size_t>(pts.size(), kMaxSamples));
    std::size_t visited = 0;
    if (pts.size() <= kMaxSamples) {
        for (const auto& p : pts) {
            ++visited;
            if ((p.flags & PointDeleted) || !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
                !std::isfinite(p.position.z))
                continue;
            sample.push_back(p.position);
        }
    } else {
        // Deterministic pseudo-random sampling avoids the periodic aliasing of a fixed stride.
        // A raster-organized cloud with N/2048 == 4 used to preserve adjacent rows, making the
        // sampled nearest-neighbour distance look as dense as the full cloud and then get
        // density-corrected a second time.
        std::unordered_set<std::size_t> chosen;
        chosen.reserve(kMaxSamples * 2);
        std::uint64_t state = 0x9e3779b97f4a7c15ull ^ static_cast<std::uint64_t>(pts.size());
        const std::size_t maxAttempts = kMaxSamples * 8;
        for (std::size_t attempt = 0; attempt < maxAttempts && sample.size() < kMaxSamples; ++attempt) {
            state = state * 6364136223846793005ull + 1442695040888963407ull;
            const std::size_t i = static_cast<std::size_t>(state % pts.size());
            if (!chosen.insert(i).second)
                continue;
            ++visited;
            const auto& p = pts[i];
            if ((p.flags & PointDeleted) || !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
                !std::isfinite(p.position.z))
                continue;
            sample.push_back(p.position);
        }
    }
    if (sample.empty())
        return out;

    const double validFraction = visited ? double(sample.size()) / double(visited) : 1.0;
    out.estimatedActivePoints = std::max<std::size_t>(sample.size(),
        static_cast<std::size_t>(std::llround(double(pts.size()) * validFraction)));

    // Robust 1%..99% bounds stop a few flying points from making every default too coarse.
    std::vector<float> xs, ys, zs;
    xs.reserve(sample.size()); ys.reserve(sample.size()); zs.reserve(sample.size());
    for (const auto& p : sample) { xs.push_back(p.x); ys.push_back(p.y); zs.push_back(p.z); }
    auto robustRange = [](std::vector<float>& v) {
        std::sort(v.begin(), v.end());
        const std::size_t n = v.size();
        const std::size_t lo = n >= 100 ? n / 100 : 0;
        const std::size_t hi = n >= 100 ? std::min(n - 1, n - 1 - n / 100) : n - 1;
        return std::max(0.0, double(v[hi]) - double(v[lo]));
    };
    const double dx = robustRange(xs), dy = robustRange(ys), dz = robustRange(zs);
    out.diagonal = std::max(1e-12, std::sqrt(dx * dx + dy * dy + dz * dz));

    // Ordered scanner clouds usually keep neighbouring pixels/points adjacent in memory.
    // Sample adjacent pairs as another strong spacing hint; median makes frame-boundary jumps
    // harmless. Imported unordered clouds simply fall back to the geometric estimators below.
    std::vector<double> orderedDistances;
    const std::size_t pairSamples = std::min<std::size_t>(4096, pts.size() > 1 ? pts.size() - 1 : 0);
    if (pairSamples > 0) {
        orderedDistances.reserve(pairSamples);
        const std::size_t pairStride = std::max<std::size_t>(1, (pts.size() - 1) / pairSamples);
        for (std::size_t i = 0; i + 1 < pts.size() && orderedDistances.size() < pairSamples; i += pairStride) {
            const auto& a = pts[i];
            const auto& b = pts[i + 1];
            if ((a.flags & PointDeleted) || (b.flags & PointDeleted)) continue;
            if (!std::isfinite(a.position.x) || !std::isfinite(a.position.y) || !std::isfinite(a.position.z) ||
                !std::isfinite(b.position.x) || !std::isfinite(b.position.y) || !std::isfinite(b.position.z)) continue;
            const double x = double(a.position.x) - b.position.x;
            const double y = double(a.position.y) - b.position.y;
            const double z = double(a.position.z) - b.position.z;
            const double d = std::sqrt(x*x + y*y + z*z);
            if (d > out.diagonal * 1e-10 && std::isfinite(d)) orderedDistances.push_back(d);
        }
    }
    double orderedPitch = std::numeric_limits<double>::infinity();
    if (!orderedDistances.empty()) {
        const auto mid = orderedDistances.begin() + static_cast<std::ptrdiff_t>(orderedDistances.size()/2);
        std::nth_element(orderedDistances.begin(), mid, orderedDistances.end());
        orderedPitch = *mid;
    }

    // Nearest-neighbour pitch on the bounded sample.  2048^2 comparisons is bounded and
    // avoids a full KD-tree build merely to open a dialog.
    std::vector<double> nearest;
    nearest.reserve(sample.size());
    for (std::size_t i = 0; i < sample.size(); ++i) {
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < sample.size(); ++j) {
            if (i == j) continue;
            const double x = double(sample[i].x) - sample[j].x;
            const double y = double(sample[i].y) - sample[j].y;
            const double z = double(sample[i].z) - sample[j].z;
            const double d2 = x*x + y*y + z*z;
            if (d2 > 0.0 && d2 < best) best = d2;
        }
        if (std::isfinite(best)) nearest.push_back(std::sqrt(best));
    }
    if (!nearest.empty()) {
        const auto mid = nearest.begin() + static_cast<std::ptrdiff_t>(nearest.size() / 2);
        std::nth_element(nearest.begin(), mid, nearest.end());
        const double sampledPitch = *mid;
        // Scanner data is a 2D surface sample.  sqrt(bbox surface / N) remains stable even
        // when the bounded random subset is much sparser than a multi-million-point cloud.
        // Use the smaller of that density estimate and the sampled NN pitch, so defaults are
        // never made artificially finer by a subset-density correction.
        const double bboxSurface = std::max(1e-24, 2.0 * (dx*dy + dx*dz + dy*dz));
        const double areaPitch = std::sqrt(bboxSurface /
                                           double(std::max<std::size_t>(1, out.estimatedActivePoints)));
        double pitch = std::min(sampledPitch, areaPitch);
        if (std::isfinite(orderedPitch)) pitch = std::min(pitch, orderedPitch);
        // Do not let duplicate/near-duplicate ordering claim a pitch more than 4x finer than
        // the global surface-density estimate.
        pitch = std::max(pitch, areaPitch * 0.25);
        out.spacing = std::max(out.diagonal * 1e-8, pitch);
    } else {
        out.spacing = std::max(out.diagonal * 1e-4, 1e-12);
    }

    out.unitsPerMillimeter = inferUnitsPerMillimeter(out.diagonal, out.spacing);
    return out;
}

inline double cloudUnitsToMillimeters(const CloudScaleEstimate& s, double units) {
    return units / std::max(1e-12, s.unitsPerMillimeter);
}

inline double millimetersToCloudUnits(const CloudScaleEstimate& s, double mm) {
    return mm * s.unitsPerMillimeter;
}

} // namespace JMEngine::processing::detail
