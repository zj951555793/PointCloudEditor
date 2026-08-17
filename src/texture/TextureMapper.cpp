#include <JMEngine/texture/TextureMapper.h>
#include "TextureMapperCuda.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <numeric>
#include <sstream>
#include <unordered_map>

#ifdef JMENGINE_USE_OPENMP
#include <omp.h>
#endif

namespace JMEngine::texture {
namespace {

constexpr float kPi = 3.14159265358979323846f;

struct V3 { float x,y,z; };

V3 sub(const Vec3f& a, const Vec3f& b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
V3 cross(const V3& a,const V3& b){ return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
float dot(const V3& a,const V3& b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
float len(const V3& a){ return std::sqrt(std::max(0.0f,dot(a,a))); }
V3 norm(V3 a){ float l=len(a); if(l>1e-12f){a.x/=l;a.y/=l;a.z/=l;} return a; }

Vec3f transformAffine(const Mat4f& m, const Vec3f& p) {
    return transformPoint(m, p);
}

Vec3f cameraCenterWorld(const Mat4f& w2c) {
    // world->camera rigid transform: C = -R^T t. Matrix storage is column-major.
    const float tx = w2c.m[12], ty = w2c.m[13], tz = w2c.m[14];
    return {
        -(w2c.m[0]*tx + w2c.m[1]*ty + w2c.m[2]*tz),
        -(w2c.m[4]*tx + w2c.m[5]*ty + w2c.m[6]*tz),
        -(w2c.m[8]*tx + w2c.m[9]*ty + w2c.m[10]*tz)
    };
}

bool project(const CameraFrame& c, const Vec3f& p, float& u, float& v, float& z) {
    const Vec3f q = transformAffine(c.worldToCamera, p);
    z = q.z;
    if (!(z > 1e-6f)) return false;
    u = c.fx*q.x/z + c.cx;
    v = c.fy*q.y/z + c.cy;
    return std::isfinite(u) && std::isfinite(v) && std::isfinite(z);
}

float edge(float ax,float ay,float bx,float by,float px,float py){ return (px-ax)*(by-ay)-(py-ay)*(bx-ax); }

void rasterDepthCamera(const TriangleMesh& mesh, const CameraFrame& c, int w, int h, std::vector<float>& out) {
    out.assign(static_cast<std::size_t>(w)*static_cast<std::size_t>(h), std::numeric_limits<float>::infinity());
    const auto cloud = mesh.vertices();
    if (!cloud || !c.image.valid()) return;
    const float sx = float(w)/float(c.image.width);
    const float sy = float(h)/float(c.image.height);
    const auto& idx = mesh.indices();

    for (std::size_t t=0;t+2<idx.size();t+=3) {
        const auto* pa=cloud->tryGet(idx[t]); const auto* pb=cloud->tryGet(idx[t+1]); const auto* pc=cloud->tryGet(idx[t+2]);
        if(!pa||!pb||!pc) continue;
        float ua,va,za,ub,vb,zb,uc,vc,zc;
        if(!project(c,pa->position,ua,va,za)||!project(c,pb->position,ub,vb,zb)||!project(c,pc->position,uc,vc,zc)) continue;
        ua*=sx; ub*=sx; uc*=sx; va*=sy; vb*=sy; vc*=sy;
        const float area=edge(ua,va,ub,vb,uc,vc);
        if(std::fabs(area)<1e-8f) continue;
        int minx=std::max(0,int(std::floor(std::min({ua,ub,uc}))));
        int maxx=std::min(w-1,int(std::ceil(std::max({ua,ub,uc}))));
        int miny=std::max(0,int(std::floor(std::min({va,vb,vc}))));
        int maxy=std::min(h-1,int(std::ceil(std::max({va,vb,vc}))));
        if(minx>maxx||miny>maxy) continue;
        for(int y=miny;y<=maxy;++y){
            for(int x=minx;x<=maxx;++x){
                float px=float(x)+0.5f, py=float(y)+0.5f;
                float w0=edge(ub,vb,uc,vc,px,py)/area;
                float w1=edge(uc,vc,ua,va,px,py)/area;
                float w2=1.0f-w0-w1;
                if(w0 < -1e-4f || w1 < -1e-4f || w2 < -1e-4f) continue;
                // z is not affine in screen space. Interpolate reciprocal depth,
                // otherwise slanted triangles produce a wrong visibility buffer and
                // camera labels become unstable around oblique surfaces.
                const float invZ = w0/za + w1/zb + w2/zc;
                if(invZ <= 1e-12f) continue;
                const float z = 1.0f/invZ;
                auto& d=out[static_cast<std::size_t>(y)*w+static_cast<std::size_t>(x)];
                if(z<d) d=z;
            }
        }
    }
}

float scoreCamera(const TriangleMesh& mesh, std::size_t triId, const CameraFrame& c,
                  const Config& cfg, const float* depth, int dw, int dh) {
    const auto cloud=mesh.vertices(); const auto& idx=mesh.indices();
    std::size_t b=triId*3u;
    if(!cloud||b+2>=idx.size()) return -1.0f;
    const auto* p0=cloud->tryGet(idx[b]); const auto* p1=cloud->tryGet(idx[b+1]); const auto* p2=cloud->tryGet(idx[b+2]);
    if(!p0||!p1||!p2) return -1.0f;
    Vec3f center{(p0->position.x+p1->position.x+p2->position.x)/3.0f,
                 (p0->position.y+p1->position.y+p2->position.y)/3.0f,
                 (p0->position.z+p1->position.z+p2->position.z)/3.0f};
    V3 n=norm(cross(sub(p1->position,p0->position),sub(p2->position,p0->position)));
    if(len(n)<1e-8f) return -1.0f;
    Vec3f cc=cameraCenterWorld(c.worldToCamera);
    V3 view=norm({cc.x-center.x,cc.y-center.y,cc.z-center.z});
    // OpenMVS-style texturing assumes an oriented mesh: a back-facing view is not a
    // valid texture observation. The old fabs() accepted the opposite side of the
    // surface and was a major source of red/black texture bleed through thin parts.
    float cosang=dot(n,view);
    if (cfg.quality == Quality::OpenMVS) cosang *= float(cfg.meshWindingSign == 0 ? 1 : cfg.meshWindingSign);
    else cosang = std::fabs(cosang);
    float minCos=std::cos(cfg.maxViewAngleDeg*kPi/180.0f);
    if(cosang<minCos) return -1.0f;
    float u,v,z; if(!project(c,center,u,v,z)) return -1.0f;
    const float mx=cfg.borderMarginRatio*float(c.image.width), my=cfg.borderMarginRatio*float(c.image.height);
    const auto inside = [&](float px, float py) {
        return px >= mx && px < float(c.image.width)-mx &&
               py >= my && py < float(c.image.height)-my;
    };
    if(!inside(u,v)) return -1.0f;

    // A triangle is only safe to assign to this photograph when ALL of its UV corners
    // lie inside the same image. Previously only the centroid was checked. A corner
    // outside the image generated a UV outside this atlas tile and sampled an adjacent
    // camera tile, which appears as random/chaotic texture patches.
    float pu[3]{}, pv[3]{}, pz[3]{};
    const Vec3f* positions[3]{&p0->position, &p1->position, &p2->position};
    for(int pi=0; pi<3; ++pi) {
        if(!project(c,*positions[pi],pu[pi],pv[pi],pz[pi]) || !inside(pu[pi],pv[pi])) return -1.0f;
    }
    if(depth && dw>0 && dh>0){
        // OpenMVS rasterizes the mesh from each view and reasons about face visibility
        // over pixels, not from one centroid sample. Approximate that robustly here with
        // the centroid plus the three edge midpoints. This avoids false holes around
        // tuft buttons, carving and depth discontinuities in a downsampled Z buffer.
        const Vec3f samples[4]{
            center,
            {(p0->position.x+p1->position.x)*0.5f,(p0->position.y+p1->position.y)*0.5f,(p0->position.z+p1->position.z)*0.5f},
            {(p1->position.x+p2->position.x)*0.5f,(p1->position.y+p2->position.y)*0.5f,(p1->position.z+p2->position.z)*0.5f},
            {(p2->position.x+p0->position.x)*0.5f,(p2->position.y+p0->position.y)*0.5f,(p2->position.z+p0->position.z)*0.5f}
        };
        int visibleSamples=0, testedSamples=0;
        for(const Vec3f& sp:samples){
            float su=0,sv=0,sz=0; if(!project(c,sp,su,sv,sz)) continue;
            const int x=std::clamp(int(su*float(dw)/float(c.image.width)),0,dw-1);
            const int y=std::clamp(int(sv*float(dh)/float(c.image.height)),0,dh-1);
            const float d=depth[static_cast<std::size_t>(y)*dw+static_cast<std::size_t>(x)];
            const float tol=std::max(cfg.visibilityTolerance,std::fabs(sz)*0.004f);
            ++testedSamples;
            if(!std::isfinite(d) || sz<=d+tol) ++visibleSamples;
        }
        if(testedSamples>0 && visibleSamples*2<testedSamples) return -1.0f;
    }
    float cxn=(u-c.cx)/std::max(1.0f,float(c.image.width)*0.5f);
    float cyn=(v-c.cy)/std::max(1.0f,float(c.image.height)*0.5f);
    float centerScore=std::max(0.0f,1.0f-0.35f*std::sqrt(cxn*cxn+cyn*cyn));

    // Prefer views that devote more source pixels to this face. This is one of the
    // important differences from angle-only greedy projection and avoids selecting a
    // very frontal but distant/low-resolution photograph.
    const float projectedArea = 0.5f * std::fabs(edge(pu[0],pv[0],pu[1],pv[1],pu[2],pv[2]));
    const float areaScore = std::clamp(std::sqrt(projectedArea) / 48.0f, 0.0f, 1.0f);
    const float minBorder = std::min({pu[0],pu[1],pu[2],
                                      float(c.image.width-1)-pu[0], float(c.image.width-1)-pu[1], float(c.image.width-1)-pu[2],
                                      pv[0],pv[1],pv[2],
                                      float(c.image.height-1)-pv[0], float(c.image.height-1)-pv[1], float(c.image.height-1)-pv[2]});
    const float borderNorm = std::max(8.0f, 0.12f*float(std::min(c.image.width,c.image.height)));
    const float borderScore = std::clamp(minBorder / borderNorm, 0.0f, 1.0f);
    const float sumW = std::max(1e-6f, cfg.angleWeight + cfg.projectedAreaWeight + cfg.imageCenterWeight + cfg.borderWeight);
    return (cfg.angleWeight*cosang + cfg.projectedAreaWeight*areaScore +
            cfg.imageCenterWeight*centerScore + cfg.borderWeight*borderScore) / sumW;
}


struct EdgeKey {
    std::uint32_t a{0}, b{0};
    bool operator==(const EdgeKey& o) const noexcept { return a == o.a && b == o.b; }
};
struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& e) const noexcept {
        return (static_cast<std::size_t>(e.a) << 32u) ^ static_cast<std::size_t>(e.b);
    }
};

std::vector<std::array<int,3>> triangleNeighbors(const TriangleMesh& mesh) {
    std::vector<std::array<int,3>> out(mesh.triangleCount(), std::array<int,3>{-1,-1,-1});
    std::vector<int> counts(mesh.triangleCount(), 0);
    std::unordered_map<EdgeKey, int, EdgeKeyHash> owner;
    owner.reserve(mesh.triangleCount() * 3u);
    const auto& idx = mesh.indices();
    for (std::size_t ti = 0; ti < mesh.triangleCount(); ++ti) {
        const std::uint32_t v[3]{idx[ti*3u], idx[ti*3u+1u], idx[ti*3u+2u]};
        for (int e = 0; e < 3; ++e) {
            EdgeKey key{std::min(v[e], v[(e+1)%3]), std::max(v[e], v[(e+1)%3])};
            const auto it = owner.find(key);
            if (it == owner.end()) { owner.emplace(key, static_cast<int>(ti)); continue; }
            const int other = it->second;
            if (other >= 0 && counts[ti] < 3 && counts[static_cast<std::size_t>(other)] < 3) {
                out[ti][counts[ti]++] = other;
                out[static_cast<std::size_t>(other)][counts[static_cast<std::size_t>(other)]++] = static_cast<int>(ti);
            }
        }
    }
    return out;
}

void smoothCameraLabels(const TriangleMesh& mesh, const std::vector<CameraFrame>& cameras,
                        const Config& cfg, std::vector<int>& best, std::vector<float>& scores) {
    if (!cfg.smoothCameraLabels || cfg.cameraLabelSmoothIterations <= 0 || best.empty()) return;
    const auto neighbors = triangleNeighbors(mesh);
    std::vector<int> next = best;
    for (int pass = 0; pass < cfg.cameraLabelSmoothIterations; ++pass) {
        next = best;
        for (std::size_t ti = 0; ti < best.size(); ++ti) {
            if (best[ti] < 0) continue;
            int labels[3]{-1,-1,-1}; int counts[3]{0,0,0}; int used = 0;
            for (int nb : neighbors[ti]) {
                if (nb < 0) continue;
                const int label = best[static_cast<std::size_t>(nb)];
                if (label < 0 || label == best[ti]) continue;
                int slot = -1;
                for (int j=0;j<used;++j) if(labels[j]==label){slot=j;break;}
                if(slot<0 && used<3){slot=used;labels[used++]=label;}
                if(slot>=0) ++counts[slot];
            }
            int winner = -1, votes = 0;
            for (int j=0;j<used;++j) if(counts[j]>votes){votes=counts[j];winner=labels[j];}
            // Only remove isolated labels: at least two adjacent triangles must agree.
            if (winner < 0 || votes < 2 || winner >= static_cast<int>(cameras.size())) continue;
            const float candidate = scoreCamera(mesh, ti, cameras[static_cast<std::size_t>(winner)], cfg, nullptr, 0, 0);
            if (candidate < 0.0f) continue;
            if (candidate + cfg.cameraLabelSwitchScoreLoss < scores[ti]) continue;
            next[ti] = winner;
            scores[ti] = candidate;
        }
        best.swap(next);
    }
}


void recoverIsolatedUnmappedFaces(const TriangleMesh& mesh, const std::vector<CameraFrame>& cameras,
                                  const Config& cfg, std::vector<int>& best, std::vector<float>& scores) {
    if (best.empty() || cameras.empty()) return;
    const auto neighbors = triangleNeighbors(mesh);
    std::vector<int> next = best;
    // Two passes are enough to close small visibility speckles without flooding genuinely
    // unobserved back/underside regions. Candidate views must already be used by a neighbour.
    for (int pass = 0; pass < 2; ++pass) {
        next = best;
        for (std::size_t ti = 0; ti < best.size(); ++ti) {
            if (best[ti] >= 0) continue;
            int candidates[3]{-1,-1,-1}; int count = 0;
            for (int nb : neighbors[ti]) {
                if (nb < 0) continue;
                const int c = best[static_cast<std::size_t>(nb)];
                if (c < 0) continue;
                bool seen=false; for(int k=0;k<count;++k) if(candidates[k]==c) seen=true;
                if(!seen && count<3) candidates[count++]=c;
            }
            float bestScore=-1.0f; int bestCamera=-1;
            for(int k=0;k<count;++k) {
                const int ci=candidates[k];
                // Deliberately omit depth here: the adjacent labelled surface is the visibility
                // prior, while projection, border and angle checks remain strict.
                const float sc=scoreCamera(mesh,ti,cameras[static_cast<std::size_t>(ci)],cfg,nullptr,0,0);
                if(sc>bestScore){bestScore=sc;bestCamera=ci;}
            }
            if(bestCamera>=0){next[ti]=bestCamera;scores[ti]=bestScore;}
        }
        best.swap(next);
    }
}


struct CameraCandidate { int camera{-1}; float score{-1.0f}; };

int estimateMeshWindingSign(const TriangleMesh& mesh,const std::vector<CameraFrame>& cameras){
    const auto cloud=mesh.vertices(); const auto& idx=mesh.indices();
    if(!cloud || cameras.empty() || mesh.triangleCount()==0) return 1;
    double vote=0.0; int samples=0;
    const std::size_t stride=std::max<std::size_t>(1,mesh.triangleCount()/2000u);
    for(std::size_t ti=0;ti<mesh.triangleCount() && samples<2000;ti+=stride){
        const Point* p0=cloud->tryGet(idx[ti*3u]); const Point* p1=cloud->tryGet(idx[ti*3u+1u]); const Point* p2=cloud->tryGet(idx[ti*3u+2u]);
        if(!p0||!p1||!p2) continue;
        const V3 n=norm(cross(sub(p1->position,p0->position),sub(p2->position,p0->position))); if(len(n)<1e-8f) continue;
        const Vec3f center{(p0->position.x+p1->position.x+p2->position.x)/3.0f,(p0->position.y+p1->position.y+p2->position.y)/3.0f,(p0->position.z+p1->position.z+p2->position.z)/3.0f};
        float strongest=0.0f,signedDot=0.0f;
        for(const auto& c:cameras){
            float u=0,v=0,z=0; if(!project(c,center,u,v,z)) continue;
            if(u<0||v<0||u>=c.image.width||v>=c.image.height) continue;
            const Vec3f cc=cameraCenterWorld(c.worldToCamera); const V3 view=norm({cc.x-center.x,cc.y-center.y,cc.z-center.z});
            const float d=dot(n,view); if(std::fabs(d)>strongest){strongest=std::fabs(d);signedDot=d;}
        }
        if(strongest>0.35f){vote += signedDot>=0?1.0:-1.0; ++samples;}
    }
    return vote>=0.0?1:-1;
}

float faceSmoothWeight(const TriangleMesh& mesh,std::size_t a,std::size_t b);

void globalOptimizeCameraLabels(const TriangleMesh& mesh, const std::vector<CameraFrame>& cameras,
                                const Config& cfg, const std::vector<float>& packedDepth,
                                int dw, int dh, std::vector<int>& best, std::vector<float>& scores) {
    if (!cfg.globalViewSelection || cfg.quality == Quality::Fast || cameras.empty() || best.empty()) return;
    const int topK = std::clamp(cfg.candidateCameraCount, 2, 8);
    const auto neighbors = triangleNeighbors(mesh);
    std::vector<std::vector<CameraCandidate>> candidates(best.size());

#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(dynamic,32)
#endif
    for (int ti=0; ti<static_cast<int>(best.size()); ++ti) {
        auto& row = candidates[static_cast<std::size_t>(ti)];
        row.reserve(static_cast<std::size_t>(topK));
        for (int ci=0; ci<static_cast<int>(cameras.size()); ++ci) {
            const float* d = packedDepth.empty() ? nullptr :
                &packedDepth[static_cast<std::size_t>(ci)*static_cast<std::size_t>(dw)*static_cast<std::size_t>(dh)];
            const float sc = scoreCamera(mesh, static_cast<std::size_t>(ti), cameras[static_cast<std::size_t>(ci)], cfg, d, dw, dh);
            if (sc < 0.0f) continue;
            auto pos = std::lower_bound(row.begin(), row.end(), sc,
                [](const CameraCandidate& a, float v){ return a.score > v; });
            row.insert(pos, CameraCandidate{ci,sc});
            if (static_cast<int>(row.size()) > topK) row.pop_back();
        }
        if (!row.empty()) { best[static_cast<std::size_t>(ti)] = row.front().camera; scores[static_cast<std::size_t>(ti)] = row.front().score; }
    }

    // ICM optimization of a Potts MRF: data term = camera quality, smoothness term =
    // shared-edge agreement. This produces connected texture patches instead of per-face speckle.
    std::vector<int> next = best;
    const int extraPasses = (cfg.quality == Quality::OpenMVS ? 6 : (cfg.quality == Quality::Ultra ? 3 : 0));
    const int passes = std::max(1, cfg.globalViewIterations + extraPasses);
    const float lambda = (cfg.quality == Quality::OpenMVS ? cfg.patchSmoothness : cfg.globalSmoothness) *
                         (cfg.quality == Quality::Ultra ? 1.25f : 1.0f);
    for (int pass=0; pass<passes; ++pass) {
        int changes=0;
        next=best;
        for (std::size_t ti=0; ti<best.size(); ++ti) {
            if (candidates[ti].empty()) continue;
            float bestEnergy=std::numeric_limits<float>::infinity();
            int bestLabel=best[ti]; float bestData=scores[ti];
            for (const auto& cand : candidates[ti]) {
                float e = 1.0f - cand.score;
                for (int nb : neighbors[ti]) {
                    if (nb < 0 || best[static_cast<std::size_t>(nb)] < 0) continue;
                    if (best[static_cast<std::size_t>(nb)] != cand.camera)
                        e += lambda * faceSmoothWeight(mesh, ti, static_cast<std::size_t>(nb));
                }
                if (e < bestEnergy) { bestEnergy=e; bestLabel=cand.camera; bestData=cand.score; }
            }
            if (bestLabel != best[ti]) ++changes;
            next[ti]=bestLabel; scores[ti]=bestData;
        }
        best.swap(next);
        if (changes==0) break;
    }
}


std::uint8_t bilinearChannel(const ImageRGB8& img,float x,float y,int ch);

float imageSharpness(const ImageRGB8& image) {
    if (!image.valid() || image.width < 3 || image.height < 3) return 0.0f;
    // Evaluate on an adaptive grid so 2 MP and 12 MP frames have comparable cost.
    const int step = std::max(1, std::min(image.width, image.height) / 240);
    double sum = 0.0, sum2 = 0.0;
    std::size_t n = 0;
    auto gray = [&](int x, int y) {
        const std::size_t i = (static_cast<std::size_t>(y) * image.width + x) * 3u;
        return 0.299 * image.pixels[i] + 0.587 * image.pixels[i + 1u] + 0.114 * image.pixels[i + 2u];
    };
    for (int y = step; y < image.height - step; y += step) {
        for (int x = step; x < image.width - step; x += step) {
            const double c = gray(x,y);
            const double lap = gray(x-step,y) + gray(x+step,y) + gray(x,y-step) + gray(x,y+step) - 4.0*c;
            sum += lap; sum2 += lap*lap; ++n;
        }
    }
    if (n < 16) return 0.0f;
    const double mean = sum / double(n);
    return static_cast<float>(std::max(0.0, sum2 / double(n) - mean*mean));
}

void rejectSeverelyBlurred(std::vector<CameraFrame>& cameras, const Config& cfg) {
    if (!cfg.rejectBlurredFrames || cameras.size() < 4) return;
    std::vector<float> sharp(cameras.size(), 0.0f), sorted;
    sorted.reserve(cameras.size());
    for (std::size_t i=0;i<cameras.size();++i) { sharp[i]=imageSharpness(cameras[i].image); if (sharp[i] > 0.0f) sorted.push_back(sharp[i]); }
    if (sorted.size() < 3) return;
    std::nth_element(sorted.begin(), sorted.begin()+sorted.size()/2, sorted.end());
    const float median = sorted[sorted.size()/2];
    if (!(median > 0.0f)) return;
    const float threshold = median * std::clamp(cfg.minRelativeSharpness, 0.02f, 0.95f);
    std::vector<CameraFrame> keep; keep.reserve(cameras.size());
    for (std::size_t i=0;i<cameras.size();++i) if (sharp[i] >= threshold) keep.push_back(std::move(cameras[i]));
    // Never let a bad threshold collapse a valid scan sequence.
    if (keep.size() >= std::max<std::size_t>(2, cameras.size()/3)) cameras.swap(keep);
}

std::array<float,3> sampleRgb(const CameraFrame& c, const Vec3f& p, bool& ok) {
    float u=0,v=0,z=0; ok=project(c,p,u,v,z);
    if (!ok || u < 1.0f || v < 1.0f || u >= c.image.width-1.0f || v >= c.image.height-1.0f) { ok=false; return {}; }
    return {float(bilinearChannel(c.image,u,v,0)), float(bilinearChannel(c.image,u,v,1)), float(bilinearChannel(c.image,u,v,2))};
}

struct GainRelation { int other{-1}; std::array<float,3> rel{}; float weight{1.0f}; };

std::vector<std::array<float,3>> seamAwareExposureGains(const TriangleMesh& mesh,
                                                         const std::vector<CameraFrame>& cameras,
                                                         const std::vector<int>& labels,
                                                         const std::vector<int>& used,
                                                         const std::vector<int>& remap,
                                                         const Config& cfg) {
    std::vector<std::array<float,3>> gains(used.size(), {1,1,1});
    if (!cfg.exposureCompensation || !cfg.seamAwareExposureCompensation || used.size() < 2) return gains;
    const auto cloud=mesh.vertices(); if(!cloud) return gains;
    const auto neighbors=triangleNeighbors(mesh); const auto& idx=mesh.indices();
    std::vector<std::vector<GainRelation>> graph(used.size());
    for (std::size_t ti=0; ti<labels.size(); ++ti) {
        const int ca=labels[ti]; if(ca<0) continue; const int ua=remap[static_cast<std::size_t>(ca)]; if(ua<0) continue;
        for(int nb:neighbors[ti]) {
            if(nb<0 || static_cast<std::size_t>(nb)<=ti) continue;
            const int cb=labels[static_cast<std::size_t>(nb)]; if(cb<0 || cb==ca) continue;
            const int ub=remap[static_cast<std::size_t>(cb)]; if(ub<0) continue;
            // Shared-edge midpoint is a real 3D correspondence observed by both patches.
            std::uint32_t shared[2]{}; int ns=0;
            for(int a=0;a<3;++a) for(int b=0;b<3;++b) if(idx[ti*3u+a]==idx[static_cast<std::size_t>(nb)*3u+b] && ns<2) shared[ns++]=idx[ti*3u+a];
            if(ns!=2) continue;
            const Point* p0=cloud->tryGet(shared[0]); const Point* p1=cloud->tryGet(shared[1]); if(!p0||!p1) continue;
            Vec3f p{(p0->position.x+p1->position.x)*0.5f,(p0->position.y+p1->position.y)*0.5f,(p0->position.z+p1->position.z)*0.5f};
            bool oka=false,okb=false; auto A=sampleRgb(cameras[static_cast<std::size_t>(ca)],p,oka); auto B=sampleRgb(cameras[static_cast<std::size_t>(cb)],p,okb); if(!oka||!okb) continue;
            std::array<float,3> delta{}; bool valid=true;
            for(int ch=0;ch<3;++ch) {
                if(A[ch]<12 || B[ch]<12 || A[ch]>245 || B[ch]>245) {valid=false;break;}
                // xb-xa = log(A/B), where corrected intensity is I*exp(x).
                delta[ch]=std::clamp(std::log(A[ch]/B[ch]), -0.35f, 0.35f);
            }
            if(!valid) continue;
            graph[static_cast<std::size_t>(ua)].push_back({ub, {-delta[0],-delta[1],-delta[2]}, 1.0f}); // xa = xb-delta
            graph[static_cast<std::size_t>(ub)].push_back({ua, { delta[0], delta[1], delta[2]}, 1.0f}); // xb = xa+delta
        }
    }
    std::vector<std::array<float,3>> x(used.size(), {0,0,0}), next=x;
    const int iters=std::clamp(cfg.exposureSolveIterations, 4, 80);
    for(int it=0; it<iters; ++it) {
        next=x;
        for(std::size_t i=0;i<x.size();++i) {
            if(graph[i].empty()) continue;
            for(int ch=0;ch<3;++ch) {
                double sum=0,w=0; for(const auto& e:graph[i]) { sum += e.weight*(x[static_cast<std::size_t>(e.other)][ch]+e.rel[ch]); w+=e.weight; }
                if(w>0) next[i][ch]=float(sum/w);
            }
        }
        // Remove gauge freedom to keep the overall sequence brightness stable.
        for(int ch=0;ch<3;++ch) { double mean=0; for(auto& v:next) mean+=v[ch]; mean/=double(next.size()); for(auto& v:next) v[ch]-=float(mean); }
        x.swap(next);
    }
    for(std::size_t i=0;i<x.size();++i) for(int ch=0;ch<3;++ch)
        gains[i][ch]=std::clamp(std::exp(x[i][ch]), cfg.exposureGainMin, cfg.exposureGainMax);
    return gains;
}

std::vector<CameraFrame> limitedCameras(const std::vector<CameraFrame>& in, int maxCount){
    std::vector<CameraFrame> out; out.reserve(std::min<std::size_t>(in.size(),std::max(1,maxCount)));
    if(in.empty()) return out;
    std::size_t n=std::min<std::size_t>(in.size(),std::max(1,maxCount));
    if(n==in.size()) return in;
    for(std::size_t i=0;i<n;++i){
        std::size_t at = (n==1)?0: (i*(in.size()-1)/(n-1));
        if(in[at].image.valid()) out.push_back(in[at]);
    }
    return out;
}

std::uint8_t bilinearChannel(const ImageRGB8& img,float x,float y,int ch){
    if(!img.valid()) return 0;
    x=std::clamp(x,0.0f,float(img.width-1)); y=std::clamp(y,0.0f,float(img.height-1));
    int x0=int(std::floor(x)), y0=int(std::floor(y)), x1=std::min(x0+1,img.width-1), y1=std::min(y0+1,img.height-1);
    float tx=x-x0, ty=y-y0;
    auto s=[&](int xx,int yy){return float(img.pixels[(static_cast<std::size_t>(yy)*img.width+xx)*3u+static_cast<std::size_t>(ch)]);};
    float a=s(x0,y0)*(1-tx)+s(x1,y0)*tx; float b=s(x0,y1)*(1-tx)+s(x1,y1)*tx;
    return static_cast<std::uint8_t>(std::clamp(a*(1-ty)+b*ty,0.0f,255.0f)+0.5f);
}

void resizeInto(const ImageRGB8& src, ImageRGB8& dst, int ox,int oy,int tw,int th, const float gain[3]){
    for(int y=0;y<th;++y) for(int x=0;x<tw;++x){
        float sx=(float(x)+0.5f)*float(src.width)/float(tw)-0.5f;
        float sy=(float(y)+0.5f)*float(src.height)/float(th)-0.5f;
        const int dstY = dst.height - 1 - (oy + y); // atlas is stored bottom-up for OpenGL/OBJ UV convention
        std::size_t d=(static_cast<std::size_t>(dstY)*dst.width+(ox+x))*3u;
        for(int c=0;c<3;++c) {
            const float value = float(bilinearChannel(src,sx,sy,c)) * gain[c];
            dst.pixels[d+c] = static_cast<std::uint8_t>(std::clamp(value, 0.0f, 255.0f) + 0.5f);
        }
    }
}


void replicateTilePadding(ImageRGB8& atlas, int ox, int oy, int tw, int th, int pad) {
    if (pad <= 0) return;
    auto copyPixel=[&](int dx,int dy,int sx,int sy){
        dx=std::clamp(dx,0,atlas.width-1); dy=std::clamp(dy,0,atlas.height-1); sx=std::clamp(sx,0,atlas.width-1); sy=std::clamp(sy,0,atlas.height-1);
        std::size_t d=(static_cast<std::size_t>(dy)*atlas.width+dx)*3u, s=(static_cast<std::size_t>(sy)*atlas.width+sx)*3u;
        atlas.pixels[d]=atlas.pixels[s]; atlas.pixels[d+1]=atlas.pixels[s+1]; atlas.pixels[d+2]=atlas.pixels[s+2];
    };
    // resizeInto stores atlas bottom-up: logical tile [ox,oy] maps to storage y = H-1-(oy+y).
    const int syTop=atlas.height-1-oy, syBottom=atlas.height-1-(oy+th-1);
    for(int x=0;x<tw;++x) for(int p=1;p<=pad;++p){ copyPixel(ox+x,syTop+p,ox+x,syTop); copyPixel(ox+x,syBottom-p,ox+x,syBottom); }
    for(int y=syBottom-pad;y<=syTop+pad;++y) for(int p=1;p<=pad;++p){ copyPixel(ox-p,y,ox,y); copyPixel(ox+tw-1+p,y,ox+tw-1,y); }
}

void imageMeanRgb(const ImageRGB8& image, double mean[3]) {
    mean[0] = mean[1] = mean[2] = 0.0;
    if (!image.valid()) return;
    const std::size_t pixels = static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
    const std::size_t stride = std::max<std::size_t>(1u, pixels / 200000u);
    std::size_t samples = 0;
    for (std::size_t p = 0; p < pixels; p += stride) {
        const std::size_t i = p * 3u;
        // Ignore nearly black / clipped-white samples, which are poor exposure references.
        const int sum = int(image.pixels[i]) + int(image.pixels[i + 1u]) + int(image.pixels[i + 2u]);
        if (sum < 24 || sum > 744) continue;
        mean[0] += image.pixels[i];
        mean[1] += image.pixels[i + 1u];
        mean[2] += image.pixels[i + 2u];
        ++samples;
    }
    if (samples == 0) return;
    for (int c = 0; c < 3; ++c) mean[c] /= double(samples);
}



struct TexturePatchInfo {
    int camera{-1};
    std::vector<std::size_t> faces;
    float minU{0}, minV{0}, maxU{0}, maxV{0};
    int srcX{0}, srcY{0}, srcW{0}, srcH{0};
    int atlasX{0}, atlasY{0}, atlasW{0}, atlasH{0};
};

std::vector<TexturePatchInfo> buildTexturePatches(const TriangleMesh& mesh,
                                                   const std::vector<CameraFrame>& cameras,
                                                   const std::vector<int>& labels,
                                                   const Config& cfg,
                                                   std::vector<int>& facePatch) {
    const auto neighbors=triangleNeighbors(mesh);
    const auto cloud=mesh.vertices();
    const auto& idx=mesh.indices();
    facePatch.assign(mesh.triangleCount(),-1);
    std::vector<TexturePatchInfo> patches;
    std::vector<std::uint8_t> seen(mesh.triangleCount(),0);
    for(std::size_t seed=0; seed<mesh.triangleCount(); ++seed){
        if(seen[seed] || labels[seed]<0) continue;
        const int camera=labels[seed];
        TexturePatchInfo patch; patch.camera=camera;
        patch.minU=patch.minV=std::numeric_limits<float>::infinity();
        patch.maxU=patch.maxV=-std::numeric_limits<float>::infinity();
        std::queue<std::size_t> q; q.push(seed); seen[seed]=1;
        while(!q.empty()){
            const std::size_t ti=q.front(); q.pop();
            patch.faces.push_back(ti);
            for(int nb:neighbors[ti]) if(nb>=0){
                const std::size_t ni=static_cast<std::size_t>(nb);
                if(!seen[ni] && labels[ni]==camera){ seen[ni]=1; q.push(ni); }
            }
        }
        const auto& cam=cameras[static_cast<std::size_t>(camera)];
        bool valid=true;
        for(std::size_t ti:patch.faces){
            for(int k=0;k<3;++k){
                const Point* p=cloud->tryGet(idx[ti*3u+static_cast<std::size_t>(k)]);
                float u=0,v=0,z=0;
                if(!p || !project(cam,p->position,u,v,z)){ valid=false; break; }
                patch.minU=std::min(patch.minU,u); patch.maxU=std::max(patch.maxU,u);
                patch.minV=std::min(patch.minV,v); patch.maxV=std::max(patch.maxV,v);
            }
            if(!valid) break;
        }
        if(!valid || !std::isfinite(patch.minU)) continue;
        const int border=std::max(1,cfg.patchBorderPixels);
        patch.srcX=std::max(0,int(std::floor(patch.minU))-border);
        patch.srcY=std::max(0,int(std::floor(patch.minV))-border);
        const int x1=std::min(cam.image.width,int(std::ceil(patch.maxU))+border+1);
        const int y1=std::min(cam.image.height,int(std::ceil(patch.maxV))+border+1);
        patch.srcW=std::max(1,x1-patch.srcX); patch.srcH=std::max(1,y1-patch.srcY);
        const int patchId=static_cast<int>(patches.size());
        for(std::size_t ti:patch.faces) facePatch[ti]=patchId;
        patches.push_back(std::move(patch));
    }
    return patches;
}

bool packTexturePatches(std::vector<TexturePatchInfo>& patches,int maxAtlas,int pad,float& scale,int& outW,int& outH){
    if(patches.empty()){outW=outH=0;scale=1;return true;}
    long double area=0; int maxW=1,maxH=1;
    for(const auto& p:patches){area+=(long double)(p.srcW+2*pad)*(p.srcH+2*pad);maxW=std::max(maxW,p.srcW+2*pad);maxH=std::max(maxH,p.srcH+2*pad);}
    scale=std::min(1.0f,static_cast<float>(std::sqrt((static_cast<long double>(maxAtlas)*maxAtlas*0.82L)/std::max<long double>(1.0L,area))));
    scale=std::min(scale,float(maxAtlas)/float(maxW)); scale=std::min(scale,float(maxAtlas)/float(maxH));
    scale=std::max(0.03f,scale);
    std::vector<int> order(patches.size()); std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](int a,int b){return patches[a].srcH>patches[b].srcH;});
    for(int attempt=0;attempt<24;++attempt){
        int x=0,y=0,rowH=0,usedW=0; bool ok=true;
        for(int id:order){
            auto& p=patches[static_cast<std::size_t>(id)];
            p.atlasW=std::max(1,int(std::lround(p.srcW*scale)));
            p.atlasH=std::max(1,int(std::lround(p.srcH*scale)));
            const int rw=p.atlasW+2*pad,rh=p.atlasH+2*pad;
            if(rw>maxAtlas||rh>maxAtlas){ok=false;break;}
            if(x+rw>maxAtlas){x=0;y+=rowH;rowH=0;}
            if(y+rh>maxAtlas){ok=false;break;}
            p.atlasX=x+pad; p.atlasY=y+pad;
            x+=rw; rowH=std::max(rowH,rh); usedW=std::max(usedW,x);
        }
        if(ok){outW=std::max(4,usedW);outH=std::max(4,y+rowH);return true;}
        scale*=0.88f;
        if(scale<0.02f) break;
    }
    return false;
}

void copyCropInto(const ImageRGB8& src,ImageRGB8& dst,int sx0,int sy0,int sw,int sh,
                  int ox,int oy,int tw,int th,const float gain[3]){
    for(int y=0;y<th;++y) for(int x=0;x<tw;++x){
        const float sx=float(sx0)+(float(x)+0.5f)*float(sw)/float(tw)-0.5f;
        const float sy=float(sy0)+(float(y)+0.5f)*float(sh)/float(th)-0.5f;
        const int dstY=dst.height-1-(oy+y);
        const std::size_t d=(static_cast<std::size_t>(dstY)*dst.width+(ox+x))*3u;
        for(int c=0;c<3;++c){
            const float v=float(bilinearChannel(src,sx,sy,c))*gain[c];
            dst.pixels[d+static_cast<std::size_t>(c)]=static_cast<std::uint8_t>(std::clamp(v,0.0f,255.0f)+0.5f);
        }
    }
}

float faceSmoothWeight(const TriangleMesh& mesh,std::size_t a,std::size_t b){
    const auto cloud=mesh.vertices(); const auto& idx=mesh.indices();
    auto normalAt=[&](std::size_t ti){
        const Point* p0=cloud->tryGet(idx[ti*3u]); const Point* p1=cloud->tryGet(idx[ti*3u+1u]); const Point* p2=cloud->tryGet(idx[ti*3u+2u]);
        if(!p0||!p1||!p2) return V3{0,0,0};
        return norm(cross(sub(p1->position,p0->position),sub(p2->position,p0->position)));
    };
    const V3 na=normalAt(a),nb=normalAt(b);
    const float c=std::clamp(dot(na,nb),-1.0f,1.0f);
    // Smooth surfaces strongly prefer one view; sharp folds are natural seam locations.
    return 0.15f+0.85f*std::max(0.0f,c);
}

std::uint32_t packRgb(std::uint8_t r,std::uint8_t g,std::uint8_t b){ return std::uint32_t(r)|(std::uint32_t(g)<<8u)|(std::uint32_t(b)<<16u)|0xff000000u; }

} // namespace

bool TextureMapper::cudaCompiled() noexcept {
#ifdef JMENGINE_TEXTURE_HAS_CUDA
    return true;
#else
    return false;
#endif
}

bool TextureMapper::cudaAvailable(std::string* reason) noexcept {
#ifdef JMENGINE_TEXTURE_HAS_CUDA
    return detail::cudaRuntimeAvailable(reason);
#else
    if(reason) *reason="CUDA backend was not compiled";
    return false;
#endif
}

Result TextureMapper::map(const TriangleMesh& mesh, const std::vector<CameraFrame>& inputCameras, const Config& config) const {
    const auto started=std::chrono::steady_clock::now();
    Result out;
    if(mesh.empty()||!mesh.vertices()||mesh.vertices()->empty()){ out.message="mesh is empty"; return out; }
    out.inputCameraCount = inputCameras.size();
    auto cameras=limitedCameras(inputCameras,config.maxKeyframes);
    cameras.erase(std::remove_if(cameras.begin(),cameras.end(),[](const CameraFrame& c){return !c.image.valid()||c.fx<=0||c.fy<=0;}),cameras.end());
    rejectSeverelyBlurred(cameras, config);
    out.acceptedCameraCount = cameras.size();
    if(cameras.empty()){ out.message="no valid texture camera frames"; return out; }
    Config runtimeConfig=config;
    if(runtimeConfig.quality==Quality::OpenMVS && runtimeConfig.meshWindingSign==0)
        runtimeConfig.meshWindingSign=estimateMeshWindingSign(mesh,cameras);
    const Config& cfg=runtimeConfig;

    int dw = std::max(32, cfg.visibilityWidth);
    int dh = std::max(32, cfg.visibilityHeight);
    std::vector<float> packedDepth;
    std::vector<int> best(mesh.triangleCount(), -1);
    std::vector<float> scores(mesh.triangleCount(), -1.0f);
    bool usedCuda = false;

    // CUDA owns both visibility depth rasterization and camera scoring. Auto falls back to the
    // complete CPU pipeline if the device is absent or a CUDA stage fails.
    if (cfg.backend != Backend::Cpu) {
#ifdef JMENGINE_TEXTURE_HAS_CUDA
        std::string why;
        if (detail::cudaRuntimeAvailable(&why)) {
            std::string err;
            if (detail::selectBestCamerasCuda(mesh, cameras, cfg, packedDepth, dw, dh, best, scores, err)) {
                usedCuda = true;
            } else if (cfg.backend == Backend::Cuda) {
                out.message = "CUDA mapping failed: " + err;
                return out;
            }
        } else if (cfg.backend == Backend::Cuda) {
            out.message = "CUDA unavailable: " + why;
            return out;
        }
#else
        if (cfg.backend == Backend::Cuda) {
            out.message = "CUDA backend not compiled";
            return out;
        }
#endif
    }

    if (!usedCuda) {
        if (cfg.buildVisibilityDepth) {
            packedDepth.resize(static_cast<std::size_t>(cameras.size()) * dw * dh);
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
            for (int ci = 0; ci < static_cast<int>(cameras.size()); ++ci) {
                std::vector<float> local;
                rasterDepthCamera(mesh, cameras[static_cast<std::size_t>(ci)], dw, dh, local);
                std::copy(local.begin(), local.end(),
                          packedDepth.begin() + static_cast<std::size_t>(ci) * dw * dh);
            }
        }
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(dynamic,64)
#endif
        for (int ti = 0; ti < static_cast<int>(mesh.triangleCount()); ++ti) {
            float bs = -1.0f;
            int bc = -1;
            for (int ci = 0; ci < static_cast<int>(cameras.size()); ++ci) {
                const float* d = packedDepth.empty()
                                     ? nullptr
                                     : &packedDepth[static_cast<std::size_t>(ci) * dw * dh];
                float score = scoreCamera(mesh, static_cast<std::size_t>(ti),
                                          cameras[static_cast<std::size_t>(ci)], cfg, d, dw, dh);
                if (score > bs) {
                    bs = score;
                    bc = ci;
                }
            }
            best[static_cast<std::size_t>(ti)] = bc;
            scores[static_cast<std::size_t>(ti)] = bs;
        }
    }
    out.backendUsed=usedCuda?Backend::Cuda:Backend::Cpu;

    // High/Ultra needs per-camera visibility for alternative labels as well, not only the
    // initial CUDA winner. Build the compact CPU depth buffers once when CUDA did not return them.
    if (cfg.globalViewSelection && cfg.quality != Quality::Fast && cfg.buildVisibilityDepth && packedDepth.empty()) {
        packedDepth.resize(static_cast<std::size_t>(cameras.size()) * dw * dh);
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int ci=0; ci<static_cast<int>(cameras.size()); ++ci) {
            std::vector<float> local; rasterDepthCamera(mesh,cameras[static_cast<std::size_t>(ci)],dw,dh,local);
            std::copy(local.begin(),local.end(),packedDepth.begin()+static_cast<std::size_t>(ci)*dw*dh);
        }
    }

    globalOptimizeCameraLabels(mesh,cameras,cfg,packedDepth,dw,dh,best,scores);

    // Per-face greedy selection tends to create A/B/A/C camera speckles on an otherwise
    // continuous surface. Regularize only isolated labels; a switch is accepted only when
    // two neighbours agree and the replacement camera remains nearly as good geometrically.
    if (cfg.quality == Quality::Fast) smoothCameraLabels(mesh, cameras, cfg, best, scores);
    recoverIsolatedUnmappedFaces(mesh, cameras, cfg, best, scores);

    // OpenMVS-style atlas generation: connected faces with the same optimized view
    // become a TexturePatch. Only the bounding rectangle actually used by that patch is
    // cropped from the ORIGINAL photograph and packed. This preserves source detail and
    // prevents an entire 1920x1200 frame from being downscaled just because many cameras
    // participate in the model.
    std::vector<int> used;
    std::vector<int> remap(cameras.size(),-1);
    for(int c:best) if(c>=0&&remap[static_cast<std::size_t>(c)]<0){
        remap[static_cast<std::size_t>(c)]=static_cast<int>(used.size()); used.push_back(c);
    }
    const bool hasUnmapped=std::any_of(best.begin(),best.end(),[](int c){return c<0;});
    if(used.empty() && !hasUnmapped){out.message="no mesh triangles are visible in the supplied camera frames";return out;}

    std::vector<int> facePatch;
    std::vector<TexturePatchInfo> patches=buildTexturePatches(mesh,cameras,best,cfg,facePatch);
    const std::size_t mappedPatchCount=patches.size();
    int fallbackPatch=-1;
    if(hasUnmapped && cfg.keepUntexturedFaces){
        TexturePatchInfo fallback; fallback.camera=-1; fallback.srcW=16; fallback.srcH=16;
        fallbackPatch=static_cast<int>(patches.size()); patches.push_back(std::move(fallback));
    }
    if(patches.empty()){out.message="texture patch generation produced no patches";return out;}

    const int pad=std::clamp(cfg.atlasPaddingPixels,0,64);
    float atlasScale=1.0f; int atlasW=0,atlasH=0;
    if(!packTexturePatches(patches,std::max(256,cfg.maxAtlasSize),pad,atlasScale,atlasW,atlasH)){
        out.message="texture patches do not fit configured atlas"; return out;
    }
    out.atlas.width=atlasW; out.atlas.height=atlasH;
    out.atlas.pixels.assign(static_cast<std::size_t>(atlasW)*static_cast<std::size_t>(atlasH)*3u,0);

    std::vector<std::array<float,3>> exposureGains=seamAwareExposureGains(mesh,cameras,best,used,remap,cfg);
    if(cfg.exposureCompensation && !cfg.seamAwareExposureCompensation && !used.empty()){
        std::vector<std::array<double,3>> means(used.size()); double global[3]{0,0,0}; std::size_t validMeans=0;
        for(std::size_t ui=0;ui<used.size();++ui){double m[3]{};imageMeanRgb(cameras[static_cast<std::size_t>(used[ui])].image,m);means[ui]={m[0],m[1],m[2]};if(m[0]>1&&m[1]>1&&m[2]>1){for(int ch=0;ch<3;++ch)global[ch]+=m[ch];++validMeans;}}
        if(validMeans){for(int ch=0;ch<3;++ch)global[ch]/=double(validMeans);for(std::size_t ui=0;ui<used.size();++ui)for(int ch=0;ch<3;++ch)if(means[ui][ch]>1)exposureGains[ui][ch]=std::clamp(float(global[ch]/means[ui][ch]),cfg.exposureGainMin,cfg.exposureGainMax);}
    }

    for(std::size_t pi=0;pi<patches.size();++pi){
        auto& patch=patches[pi];
        if(patch.camera<0){
            // OpenMVS also keeps faces invisible in all input views in a dedicated constant-color patch.
            for(int y=0;y<patch.atlasH;++y) for(int x=0;x<patch.atlasW;++x){
                const int yy=out.atlas.height-1-(patch.atlasY+y);
                const std::size_t q=(static_cast<std::size_t>(yy)*out.atlas.width+(patch.atlasX+x))*3u;
                out.atlas.pixels[q]=190;out.atlas.pixels[q+1]=190;out.atlas.pixels[q+2]=190;
            }
            replicateTilePadding(out.atlas,patch.atlasX,patch.atlasY,patch.atlasW,patch.atlasH,pad);
            continue;
        }
        const int ui=remap[static_cast<std::size_t>(patch.camera)];
        if(ui<0) continue;
        const auto& cam=cameras[static_cast<std::size_t>(patch.camera)];
        copyCropInto(cam.image,out.atlas,patch.srcX,patch.srcY,patch.srcW,patch.srcH,
                     patch.atlasX,patch.atlasY,patch.atlasW,patch.atlasH,exposureGains[static_cast<std::size_t>(ui)].data());
        replicateTilePadding(out.atlas,patch.atlasX,patch.atlasY,patch.atlasW,patch.atlasH,pad);
    }

    Vec2f fallbackUv{0.5f,0.5f};
    if(fallbackPatch>=0){
        const auto& p=patches[static_cast<std::size_t>(fallbackPatch)];
        fallbackUv={(float(p.atlasX)+0.5f*float(p.atlasW))/float(atlasW),
                    1.0f-(float(p.atlasY)+0.5f*float(p.atlasH))/float(atlasH)};
    }
    PointCloud::Container verts; verts.reserve(mesh.triangleCount()*3u);
    out.indices.reserve(mesh.triangleCount()*3u); out.texcoords.reserve(mesh.triangleCount()*3u);
    // Keep one camera assignment per ORIGINAL input triangle. -1 means unmapped.
    out.triangleCameraIds.assign(mesh.triangleCount(), -1);
    std::size_t mappedTriangleCount = 0;
    const auto cloud=mesh.vertices(); const auto& idx=mesh.indices();
    for(std::size_t ti=0;ti<mesh.triangleCount();++ti){
        const int ci=best[ti];
        std::uint32_t base=static_cast<std::uint32_t>(verts.size()); bool valid=true;
        Point temp[3]; Vec2f tuv[3];
        if(ci<0){
            for(int k=0;k<3;++k){
                const Point* src=cloud->tryGet(idx[ti*3u+static_cast<std::size_t>(k)]); if(!src){valid=false;break;}
                temp[k]=*src; tuv[k]=fallbackUv;
            }
        } else {
            const int ui=remap[static_cast<std::size_t>(ci)];
            const int pi=(ti<facePatch.size()?facePatch[ti]:-1);
            if(ui<0 || pi<0 || static_cast<std::size_t>(pi)>=patches.size()){valid=false;}
            else {
                const auto& patch=patches[static_cast<std::size_t>(pi)];
                const auto& cam=cameras[static_cast<std::size_t>(ci)];
                const float sx=float(patch.atlasW)/float(std::max(1,patch.srcW));
                const float sy=float(patch.atlasH)/float(std::max(1,patch.srcH));
                for(int k=0;k<3;++k){
                    const Point* src=cloud->tryGet(idx[ti*3u+static_cast<std::size_t>(k)]); if(!src){valid=false;break;} temp[k]=*src;
                    float u,v,z; if(!project(cam,src->position,u,v,z)){valid=false;break;}
                    // scoreCamera already validated the complete triangle against this source image.
                    // Patch UVs stay in source-pixel coordinates until the final atlas transform.
                    const float au=float(patch.atlasX)+(u-float(patch.srcX))*sx;
                    const float av=float(patch.atlasY)+(v-float(patch.srcY))*sy;
                    tuv[k]={std::clamp(au/float(atlasW),0.0f,1.0f),std::clamp(1.0f-av/float(atlasH),0.0f,1.0f)};
                    if(cfg.bakePreviewVertexColors){
                        const auto& gg=exposureGains[static_cast<std::size_t>(ui)];
                        const auto rr=float(bilinearChannel(cam.image,u,v,0))*gg[0];
                        const auto gr=float(bilinearChannel(cam.image,u,v,1))*gg[1];
                        const auto bb=float(bilinearChannel(cam.image,u,v,2))*gg[2];
                        temp[k].rgba=packRgb(static_cast<std::uint8_t>(std::clamp(rr,0.0f,255.0f)),static_cast<std::uint8_t>(std::clamp(gr,0.0f,255.0f)),static_cast<std::uint8_t>(std::clamp(bb,0.0f,255.0f)));
                    }
                }
            }
        }
        if(!valid) continue;
        for(int k=0;k<3;++k){ verts.push_back(temp[k]); out.texcoords.push_back(tuv[k]); out.indices.push_back(base+static_cast<std::uint32_t>(k)); }
        if(ci>=0){ out.triangleCameraIds[ti] = cameras[static_cast<std::size_t>(ci)].frameId; ++mappedTriangleCount; }
    }
    if(verts.empty()){ out.message="texture mapping produced no valid triangles"; return out; }
    out.vertices=std::make_shared<PointCloud>(std::move(verts)); out.ok=true;
    out.usedCameraCount=used.size(); out.mappedTriangleCount=mappedTriangleCount;
    out.unmappedTriangleCount=mesh.triangleCount()-mappedTriangleCount;
    out.texturePatchCount=mappedPatchCount;
    out.elapsedMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    std::ostringstream ss; ss << "texture mapped " << mappedTriangleCount << "/" << mesh.triangleCount()
                            << " triangles with " << used.size()
                            << " cameras using " << (usedCuda?"CUDA":"CPU") << " backend, quality="
                            << (cfg.quality==Quality::OpenMVS?"OpenMVS":(cfg.quality==Quality::Ultra?"Ultra":(cfg.quality==Quality::High?"High":"Fast")))
                            << ", patches=" << mappedPatchCount << ", atlasScale=" << atlasScale; out.message=ss.str();
    return out;
}

bool saveObj(const Result& result, const std::string& path, std::string* message,
             const TriangleMesh* activeTopology){
    if (!result.ok || !result.vertices || result.indices.empty() ||
        result.texcoords.size() != result.vertices->size() || !result.atlas.valid()) {
        if (message) *message = "invalid texture result";
        return false;
    }
    auto dot=path.find_last_of('.'); std::string stem=(dot==std::string::npos)?path:path.substr(0,dot); std::string obj=stem+".obj", mtl=stem+".mtl", tex=stem+"_texture.tga";
    auto slash=stem.find_last_of("/\\"); std::string base=(slash==std::string::npos)?stem:stem.substr(slash+1);
    std::ofstream tf(tex, std::ios::binary);
    if (!tf) { if (message) *message = "cannot open texture output"; return false; }
    unsigned char header[18]{};
    header[2] = 2; // uncompressed true-color
    header[12] = static_cast<unsigned char>(result.atlas.width & 0xff);
    header[13] = static_cast<unsigned char>((result.atlas.width >> 8) & 0xff);
    header[14] = static_cast<unsigned char>(result.atlas.height & 0xff);
    header[15] = static_cast<unsigned char>((result.atlas.height >> 8) & 0xff);
    header[16] = 24;
    header[17] = 0x00; // bottom-left origin; atlas storage already matches OBJ/OpenGL V convention
    tf.write(reinterpret_cast<const char*>(header), sizeof(header));
    for (std::size_t i = 0; i < result.atlas.pixels.size(); i += 3u) {
        const unsigned char bgr[3]{result.atlas.pixels[i + 2u], result.atlas.pixels[i + 1u], result.atlas.pixels[i]};
        tf.write(reinterpret_cast<const char*>(bgr), 3);
    }
    std::ofstream mf(mtl); if(!mf){if(message)*message="cannot open mtl output";return false;} mf<<"newmtl material0\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd "<<base<<"_texture.tga\n";
    std::ofstream of(obj); if(!of){if(message)*message="cannot open obj output";return false;} of<<"mtllib "<<base<<".mtl\nusemtl material0\n";
    for(const auto& p:result.vertices->points()) of<<"v "<<p.position.x<<" "<<p.position.y<<" "<<p.position.z<<"\n";
    for(const auto& uv:result.texcoords) of<<"vt "<<uv.x<<" "<<uv.y<<"\n";
    if(activeTopology && activeTopology->triangleCount()!=result.indices.size()/3u){
        if(message)*message="textured mesh topology changed; remap texture before export";
        return false;
    }
    for(std::size_t i=0;i+2<result.indices.size();i+=3){
        const std::size_t ti=i/3u;
        if(activeTopology && !activeTopology->triangleActive(static_cast<TriangleId>(ti))) continue;
        auto a=result.indices[i]+1,b=result.indices[i+1]+1,c=result.indices[i+2]+1;
        of<<"f "<<a<<"/"<<a<<" "<<b<<"/"<<b<<" "<<c<<"/"<<c<<"\n";
    }
    if (message) *message = "saved textured OBJ/MTL/TGA";
    return true;
}

} // namespace JMEngine::texture
