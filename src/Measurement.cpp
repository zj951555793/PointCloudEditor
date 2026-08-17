#include <JMEngine/Measurement.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace JMEngine {
namespace {
inline Vec3f sub(const Vec3f& a,const Vec3f& b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3f cross(const Vec3f& a,const Vec3f& b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double dotd(const Vec3f& a,const Vec3f& b){return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z;}
inline double norm(const Vec3f& a){return std::sqrt(dotd(a,a));}
inline std::uint64_t edgeKey(std::uint32_t a,std::uint32_t b){if(a>b)std::swap(a,b);return (std::uint64_t(a)<<32)|b;}
}

DistanceMeasurement measureDistance(const Vec3f& a, const Vec3f& b) noexcept {
    const Vec3f d=sub(b,a); return {a,b,norm(d)};
}
AngleMeasurement measureAngle(const Vec3f& a, const Vec3f& vertex, const Vec3f& c) noexcept {
    const Vec3f u=sub(a,vertex),v=sub(c,vertex); const double un=norm(u),vn=norm(v); double r=0.0;
    if(un>1e-12&&vn>1e-12)r=std::acos(std::clamp(dotd(u,v)/(un*vn),-1.0,1.0));
    constexpr double k=57.2957795130823208768; return {a,vertex,c,r,r*k};
}

SurfaceAreaMeasurement measureSurfaceArea(const TriangleMesh& mesh,const MeshMeasureOptions& options) noexcept {
    SurfaceAreaMeasurement out; const auto cloud=mesh.vertices(); if(!cloud)return out;
    const auto& idx=mesh.indices(); const auto& flags=mesh.triangleFlags();
    for(std::size_t t=0;t+2<idx.size();t+=3){
        const std::size_t tid=t/3; if(tid<flags.size()&&(flags[tid]&TriangleDeleted))continue;
        const auto* a=cloud->tryGet(idx[t]); const auto* b=cloud->tryGet(idx[t+1]); const auto* c=cloud->tryGet(idx[t+2]);
        if(!a||!b||!c||(a->flags&PointDeleted)||(b->flags&PointDeleted)||(c->flags&PointDeleted)){++out.degenerateTriangleCount;continue;}
        const double area=0.5*norm(cross(sub(b->position,a->position),sub(c->position,a->position)));
        const double eps=std::max(options.minTriangleArea,1e-18);
        if(!(area>eps)||!std::isfinite(area)){++out.degenerateTriangleCount;continue;}
        out.area+=area; ++out.triangleCount;
    }
    out.valid=out.triangleCount>0&&std::isfinite(out.area); return out;
}

VolumeMeasurement measureVolume(const TriangleMesh& mesh,const MeshMeasureOptions& options) noexcept {
    VolumeMeasurement out; const auto cloud=mesh.vertices(); if(!cloud)return out;
    const auto& idx=mesh.indices(); const auto& flags=mesh.triangleFlags();
    std::unordered_map<std::uint64_t,std::uint32_t> edges; edges.reserve(idx.size());
    long double sum=0.0L;
    for(std::size_t t=0;t+2<idx.size();t+=3){
        const std::size_t tid=t/3; if(tid<flags.size()&&(flags[tid]&TriangleDeleted))continue;
        const std::uint32_t ia=idx[t],ib=idx[t+1],ic=idx[t+2];
        const auto* a=cloud->tryGet(ia); const auto* b=cloud->tryGet(ib); const auto* c=cloud->tryGet(ic);
        if(!a||!b||!c||(a->flags&PointDeleted)||(b->flags&PointDeleted)||(c->flags&PointDeleted)){++out.degenerateTriangleCount;continue;}
        const double area=0.5*norm(cross(sub(b->position,a->position),sub(c->position,a->position)));
        const double eps=std::max(options.minTriangleArea,1e-18);
        if(!(area>eps)||!std::isfinite(area)){++out.degenerateTriangleCount;continue;}
        ++edges[edgeKey(ia,ib)]; ++edges[edgeKey(ib,ic)]; ++edges[edgeKey(ic,ia)];
        sum += (long double)dotd(a->position,cross(b->position,c->position))/6.0L; ++out.triangleCount;
    }
    for(const auto& e:edges){if(e.second==1)++out.boundaryEdgeCount;else if(e.second!=2)++out.nonManifoldEdgeCount;}
    out.signedVolume=(double)sum; out.volume=std::abs(out.signedVolume);
    const bool topoOk=out.boundaryEdgeCount==0&&out.nonManifoldEdgeCount==0;
    out.valid=out.triangleCount>0&&std::isfinite(out.volume)&&(!options.requireWatertightForVolume||topoOk);
    return out;
}
} // namespace JMEngine
