#include <pceditor/Alignment.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace pceditor {
namespace {
Vec3f add(Vec3f a,Vec3f b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3f sub(Vec3f a,Vec3f b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3f mul(Vec3f a,float s){return {a.x*s,a.y*s,a.z*s};}
double dotd(Vec3f a,Vec3f b){return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z;}
Vec3f cross(Vec3f a,Vec3f b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
float norm(Vec3f a){return float(std::sqrt(dotd(a,a)));}
bool normalize(Vec3f& a){float n=norm(a);if(n<1e-10f)return false;a=mul(a,1.f/n);return true;}
Mat4f multiply(const Mat4f& a,const Mat4f& b){Mat4f o{};o.m.fill(0.f);for(int c=0;c<4;++c)for(int r=0;r<4;++r)for(int k=0;k<4;++k)o.m[c*4+r]+=a.m[k*4+r]*b.m[c*4+k];return o;}
Vec3f rotate(const float r[9],Vec3f p){return {r[0]*p.x+r[1]*p.y+r[2]*p.z,r[3]*p.x+r[4]*p.y+r[5]*p.z,r[6]*p.x+r[7]*p.y+r[8]*p.z};}
Mat4f fromRt(const float r[9],Vec3f t){Mat4f m=Mat4f::identity();m.m[0]=r[0];m.m[4]=r[1];m.m[8]=r[2];m.m[1]=r[3];m.m[5]=r[4];m.m[9]=r[5];m.m[2]=r[6];m.m[6]=r[7];m.m[10]=r[8];m.m[12]=t.x;m.m[13]=t.y;m.m[14]=t.z;return m;}

bool basis(const std::array<Vec3f,3>& p,Vec3f& x,Vec3f& y,Vec3f& z){x=sub(p[1],p[0]);if(!normalize(x))return false;Vec3f v=sub(p[2],p[0]);z=cross(x,v);if(!normalize(z))return false;y=cross(z,x);return normalize(y);}

struct Bounds{Vec3f mn{},mx{};bool valid=false;};
Bounds boundsOf(const std::vector<Vec3f>& p){Bounds b;if(p.empty())return b;b.mn=b.mx=p[0];for(auto q:p){b.mn.x=std::min(b.mn.x,q.x);b.mn.y=std::min(b.mn.y,q.y);b.mn.z=std::min(b.mn.z,q.z);b.mx.x=std::max(b.mx.x,q.x);b.mx.y=std::max(b.mx.y,q.y);b.mx.z=std::max(b.mx.z,q.z);}b.valid=true;return b;}
float diag(const Bounds& b){return norm(sub(b.mx,b.mn));}

struct Key{int x,y,z;bool operator==(const Key&o)const{return x==o.x&&y==o.y&&z==o.z;}};
struct KeyHash{std::size_t operator()(const Key&k)const noexcept{std::uint64_t h=1469598103934665603ull;auto f=[&](int v){h^=(std::uint32_t)v;h*=1099511628211ull;};f(k.x);f(k.y);f(k.z);return (std::size_t)h;}};
Key cell(Vec3f p,float s){return {(int)std::floor(p.x/s),(int)std::floor(p.y/s),(int)std::floor(p.z/s)};}

std::vector<Vec3f> sampleCloud(const PointCloud& c,const Mat4f& tf,float voxel,std::size_t maxPts){
    std::unordered_map<Key,Vec3f,KeyHash> vox; vox.reserve(std::min<std::size_t>(c.size(),maxPts*2));
    if(!(voxel>0))voxel=1e-6f;
    const std::size_t stride=std::max<std::size_t>(1,c.size()/std::max<std::size_t>(maxPts*4,1));
    for(std::size_t i=0;i<c.size();i+=stride){const auto&p=c.points()[i];if(p.flags&PointDeleted)continue;Vec3f q=transformPoint(tf,p.position);Key k=cell(q,voxel);if(vox.find(k)==vox.end())vox.emplace(k,q);if(vox.size()>=maxPts)break;}
    std::vector<Vec3f> out;out.reserve(vox.size());for(auto&kv:vox)out.push_back(kv.second);return out;
}

Vec3f centroid(const std::vector<Vec3f>& p){long double x=0,y=0,z=0;for(auto q:p){x+=q.x;y+=q.y;z+=q.z;}const long double n=std::max<std::size_t>(1,p.size());return {(float)(x/n),(float)(y/n),(float)(z/n)};}

// Jacobi 对称 3x3 特征分解，列向量为特征向量，按特征值降序。
bool pcaFrame(const std::vector<Vec3f>& p,Vec3f axes[3],Vec3f& c){if(p.size()<6)return false;c=centroid(p);double a[3][3]{};for(auto q:p){double x=q.x-c.x,y=q.y-c.y,z=q.z-c.z;a[0][0]+=x*x;a[0][1]+=x*y;a[0][2]+=x*z;a[1][1]+=y*y;a[1][2]+=y*z;a[2][2]+=z*z;}a[1][0]=a[0][1];a[2][0]=a[0][2];a[2][1]=a[1][2];double v[3][3]={{1,0,0},{0,1,0},{0,0,1}};for(int it=0;it<24;++it){int p0=0,q0=1;double m=std::abs(a[0][1]);if(std::abs(a[0][2])>m){m=std::abs(a[0][2]);p0=0;q0=2;}if(std::abs(a[1][2])>m){m=std::abs(a[1][2]);p0=1;q0=2;}if(m<1e-12)break;double phi=.5*std::atan2(2*a[p0][q0],a[q0][q0]-a[p0][p0]),cs=std::cos(phi),sn=std::sin(phi);double app=cs*cs*a[p0][p0]-2*sn*cs*a[p0][q0]+sn*sn*a[q0][q0],aqq=sn*sn*a[p0][p0]+2*sn*cs*a[p0][q0]+cs*cs*a[q0][q0];for(int k=0;k<3;++k)if(k!=p0&&k!=q0){double akp=a[k][p0],akq=a[k][q0];a[k][p0]=a[p0][k]=cs*akp-sn*akq;a[k][q0]=a[q0][k]=sn*akp+cs*akq;}a[p0][p0]=app;a[q0][q0]=aqq;a[p0][q0]=a[q0][p0]=0;for(int k=0;k<3;++k){double vip=v[k][p0],viq=v[k][q0];v[k][p0]=cs*vip-sn*viq;v[k][q0]=sn*vip+cs*viq;}}
    std::array<int,3> o{0,1,2};std::sort(o.begin(),o.end(),[&](int i,int j){return a[i][i]>a[j][j];});for(int n=0;n<3;++n){axes[n]={(float)v[0][o[n]],(float)v[1][o[n]],(float)v[2][o[n]]};normalize(axes[n]);}if(dotd(cross(axes[0],axes[1]),axes[2])<0)axes[2]=mul(axes[2],-1);return true;}

Mat4f frameTransform(const Vec3f s[3],Vec3f cs,const Vec3f t[3],Vec3f ct,const int signs[3]){float R[9]{};Vec3f ss[3]={mul(s[0],(float)signs[0]),mul(s[1],(float)signs[1]),mul(s[2],(float)signs[2])};for(int i=0;i<3;++i){const float tv[3]={t[i].x,t[i].y,t[i].z};const float sv[3]={ss[i].x,ss[i].y,ss[i].z};for(int r=0;r<3;++r)for(int c=0;c<3;++c)R[r*3+c]+=tv[r]*sv[c];}Vec3f tr=sub(ct,rotate(R,cs));return fromRt(R,tr);}

struct Grid{float s;std::unordered_map<Key,std::vector<int>,KeyHash> m;const std::vector<Vec3f>*p{};explicit Grid(const std::vector<Vec3f>&pts,float cellSize):s(cellSize),p(&pts){m.reserve(pts.size()*2);for(int i=0;i<(int)pts.size();++i)m[cell(pts[i],s)].push_back(i);}bool nearest(Vec3f q,float maxD,Vec3f& out,float& d2)const{Key k=cell(q,s);int rad=std::max(1,(int)std::ceil(maxD/s));d2=maxD*maxD;bool ok=false;for(int z=-rad;z<=rad;++z)for(int y=-rad;y<=rad;++y)for(int x=-rad;x<=rad;++x){auto it=m.find({k.x+x,k.y+y,k.z+z});if(it==m.end())continue;for(int id:it->second){Vec3f d=sub((*p)[id],q);float dd=(float)dotd(d,d);if(dd<d2){d2=dd;out=(*p)[id];ok=true;}}}return ok;}};

Mat4f rigidFit(const std::vector<Vec3f>& s,const std::vector<Vec3f>& t){Vec3f cs=centroid(s),ct=centroid(t);double Sxx=0,Sxy=0,Sxz=0,Syx=0,Syy=0,Syz=0,Szx=0,Szy=0,Szz=0;for(std::size_t i=0;i<s.size();++i){Vec3f a=sub(s[i],cs),b=sub(t[i],ct);Sxx+=a.x*b.x;Sxy+=a.x*b.y;Sxz+=a.x*b.z;Syx+=a.y*b.x;Syy+=a.y*b.y;Syz+=a.y*b.z;Szx+=a.z*b.x;Szy+=a.z*b.y;Szz+=a.z*b.z;}double N[4][4]={{Sxx+Syy+Szz,Syz-Szy,Szx-Sxz,Sxy-Syx},{Syz-Szy,Sxx-Syy-Szz,Sxy+Syx,Szx+Sxz},{Szx-Sxz,Sxy+Syx,-Sxx+Syy-Szz,Syz+Szy},{Sxy-Syx,Szx+Sxz,Syz+Szy,-Sxx-Syy+Szz}};double q[4]={1,0,0,0};for(int it=0;it<50;++it){double nq[4]{};for(int r=0;r<4;++r)for(int c=0;c<4;++c)nq[r]+=N[r][c]*q[c];double n=std::sqrt(nq[0]*nq[0]+nq[1]*nq[1]+nq[2]*nq[2]+nq[3]*nq[3]);if(n<1e-20)break;for(int i=0;i<4;++i)q[i]=nq[i]/n;}double w=q[0],x=q[1],y=q[2],z=q[3];float R[9]={(float)(1-2*(y*y+z*z)),(float)(2*(x*y-z*w)),(float)(2*(x*z+y*w)),(float)(2*(x*y+z*w)),(float)(1-2*(x*x+z*z)),(float)(2*(y*z-x*w)),(float)(2*(x*z-y*w)),(float)(2*(y*z+x*w)),(float)(1-2*(x*x+y*y))};return fromRt(R,sub(ct,rotate(R,cs)));}

struct IcpEval{Mat4f tf{Mat4f::identity()};float rms=std::numeric_limits<float>::infinity();float ratio=0;std::size_t count=0;int iterations=0;};
IcpEval icp(const std::vector<Vec3f>& src,const std::vector<Vec3f>& tgt,Mat4f initial,float maxD,int maxIter,float trim){IcpEval out;out.tf=initial;Grid grid(tgt,std::max(maxD*.5f,1e-7f));float prev=std::numeric_limits<float>::infinity();for(int it=0;it<maxIter;++it){struct C{Vec3f s,t;float d2;};std::vector<C> cs;cs.reserve(src.size());for(auto p:src){Vec3f q=transformPoint(out.tf,p),nn;float d2;if(grid.nearest(q,maxD,nn,d2))cs.push_back({q,nn,d2});}if(cs.size()<6)break;std::size_t keep=std::max<std::size_t>(6,(std::size_t)(cs.size()*std::clamp(trim,0.2f,1.0f)));std::nth_element(cs.begin(),cs.begin()+keep-1,cs.end(),[](auto&a,auto&b){return a.d2<b.d2;});cs.resize(keep);std::vector<Vec3f>a,b;a.reserve(keep);b.reserve(keep);double sum=0;for(auto&c:cs){a.push_back(c.s);b.push_back(c.t);sum+=c.d2;}float rms=(float)std::sqrt(sum/keep);Mat4f delta=rigidFit(a,b);out.tf=multiply(delta,out.tf);out.rms=rms;out.count=keep;out.ratio=(float)keep/(float)src.size();out.iterations=it+1;if(std::abs(prev-rms)<std::max(1e-7f,rms*1e-5f))break;prev=rms;}return out;}
}

AlignmentResult alignThreePoints(const std::array<Vec3f,3>& source,const std::array<Vec3f,3>& target) noexcept {AlignmentResult out;Vec3f sx,sy,sz,tx,ty,tz;if(!basis(source,sx,sy,sz)||!basis(target,tx,ty,tz))return out;const float S[9]={sx.x,sy.x,sz.x,sx.y,sy.y,sz.y,sx.z,sy.z,sz.z};const float T[9]={tx.x,ty.x,tz.x,tx.y,ty.y,tz.y,tx.z,ty.z,tz.z};float R[9]{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)R[i*3+j]+=T[i*3+k]*S[j*3+k];Vec3f cs=mul(add(add(source[0],source[1]),source[2]),1.f/3),ct=mul(add(add(target[0],target[1]),target[2]),1.f/3);Mat4f m=fromRt(R,sub(ct,rotate(R,cs)));double sum=0;for(int i=0;i<3;++i){Vec3f d=sub(transformPoint(m,source[i]),target[i]);sum+=dotd(d,d);}out.success=true;out.transform=m;out.rmsError=(float)std::sqrt(sum/3);return out;}

AutoAlignmentResult alignPointClouds(const PointCloud& source,const Mat4f& sourceTransform,const PointCloud& target,const Mat4f& targetTransform,const AutoAlignmentOptions& o){AutoAlignmentResult out;if(source.empty()||target.empty()){out.status=AutoAlignmentStatus::InvalidInput;return out;}std::vector<Vec3f> probe;probe.reserve(std::min<std::size_t>(target.size(),50000));std::size_t stride=std::max<std::size_t>(1,target.size()/50000);for(std::size_t i=0;i<target.size();i+=stride){auto&p=target.points()[i];if(!(p.flags&PointDeleted))probe.push_back(transformPoint(targetTransform,p.position));}if(probe.size()<20){out.status=AutoAlignmentStatus::NotEnoughPoints;return out;}float d=diag(boundsOf(probe));if(!(d>1e-8f)){out.status=AutoAlignmentStatus::InvalidInput;return out;}float coarse=o.coarseVoxelSize>0?o.coarseVoxelSize:d*0.02f;float fine=o.fineVoxelSize>0?o.fineVoxelSize:d*0.006f;float maxD=o.maxCorrespondenceDistance>0?o.maxCorrespondenceDistance:d*0.08f;float accept=o.maxAcceptedRms>0?o.maxAcceptedRms:d*0.01f;auto sCoarse=sampleCloud(source,sourceTransform,coarse,std::max<std::size_t>(2000,o.maxSamplePoints/4));auto tCoarse=sampleCloud(target,targetTransform,coarse,std::max<std::size_t>(2000,o.maxSamplePoints/4));if(sCoarse.size()<20||tCoarse.size()<20){out.status=AutoAlignmentStatus::NotEnoughPoints;return out;}Vec3f sa[3],ta[3],cs,ct;if(!pcaFrame(sCoarse,sa,cs)||!pcaFrame(tCoarse,ta,ct)){out.status=AutoAlignmentStatus::NotEnoughPoints;return out;}const int signs[4][3]={{1,1,1},{1,-1,-1},{-1,1,-1},{-1,-1,1}};IcpEval best;for(auto&sg:signs){Mat4f init=frameTransform(sa,cs,ta,ct,sg);auto e=icp(sCoarse,tCoarse,init,maxD,o.coarseIterations,o.trimFraction);if(e.count>best.count/2&&e.rms<best.rms)best=e;}if(best.count<6||!std::isfinite(best.rms)){out.status=AutoAlignmentStatus::NoCorrespondence;return out;}auto sFine=sampleCloud(source,sourceTransform,fine,o.maxSamplePoints);auto tFine=sampleCloud(target,targetTransform,fine,o.maxSamplePoints);auto fin=icp(sFine,tFine,best.tf,std::max(maxD*.35f,fine*3.f),o.fineIterations,o.trimFraction);out.transform=fin.tf;out.rmsError=fin.rms;out.inlierRatio=fin.ratio;out.correspondenceCount=fin.count;out.iterations=best.iterations+fin.iterations;if(fin.count<6||!std::isfinite(fin.rms)){out.status=AutoAlignmentStatus::NotConverged;return out;}if(fin.ratio<o.minInlierRatio||fin.rms>accept){out.status=AutoAlignmentStatus::QualityRejected;return out;}out.status=AutoAlignmentStatus::Success;out.success=true;return out;}

} // namespace pceditor
