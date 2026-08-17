#include "JMEngine/ObjModelLoader.h"
#include <JMEngine/processing/Parallel.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef JMENGINE_USE_OPENMP
#include <omp.h>
#endif

namespace JMEngine {
namespace {

struct FaceRef {
    int v{-1};
    int vt{-1};
    int vn{-1};
};

inline void skipSpace(const char*& p) {
    while (*p == ' ' || *p == '\t')
        ++p;
}

bool parseFloatFast(const char*& p, float& out) {
    skipSpace(p);
    if (*p == '\0' || *p == '\r' || *p == '\n')
        return false;
    char* end = nullptr;
    out = std::strtof(p, &end);
    if (end == p)
        return false;
    p = end;
    return true;
}

bool parseIntFast(const char*& p, int& out) {
    if (*p == '\0')
        return false;
    char* end = nullptr;
    const long value = std::strtol(p, &end, 10);
    if (end == p)
        return false;
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
        return false;
    }
    out = static_cast<int>(value);
    p = end;
    return true;
}

int resolveIndex(int rawIndex, int count) {
    if (rawIndex > 0)
        return rawIndex - 1;
    if (rawIndex < 0)
        return count + rawIndex;
    return -1;
}

FaceRef parseFaceToken(const char*& p, int vc, int tc, int nc) {
    FaceRef out;
    int raw = 0;
    if (!parseIntFast(p, raw))
        return out;
    out.v = resolveIndex(raw, vc);

    if (*p != '/')
        return out;
    ++p;
    if (*p != '/') {
        if (parseIntFast(p, raw))
            out.vt = resolveIndex(raw, tc);
    }
    if (*p == '/') {
        ++p;
        if (parseIntFast(p, raw))
            out.vn = resolveIndex(raw, nc);
    }
    return out;
}

Vec3f normalizeVec(const Vec3f& v) {
    const float len2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (len2 <= 1.0e-24f)
        return {};
    const float inv = 1.0f / std::sqrt(len2);
    return {v.x * inv, v.y * inv, v.z * inv};
}

std::uint8_t objColorToByte(float value) {
    if (value >= 0.0f && value <= 1.0f)
        value *= 255.0f;
    value = std::clamp(value, 0.0f, 255.0f);
    return static_cast<std::uint8_t>(std::lround(value));
}

std::uint32_t packRgba(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 255u) {
    return static_cast<std::uint32_t>(r) | (static_cast<std::uint32_t>(g) << 8u) |
           (static_cast<std::uint32_t>(b) << 16u) | (static_cast<std::uint32_t>(a) << 24u);
}

bool startsWithToken(const std::string& line, const char* token) {
    const std::size_t n = std::char_traits<char>::length(token);
    if (line.size() < n || line.compare(0, n, token) != 0)
        return false;
    return line.size() == n || std::isspace(static_cast<unsigned char>(line[n])) != 0;
}

// 第一遍只需要知道一个 face 有多少个顶点，不需要保存 token。
// 这样 600 万面时不会产生数百万个 vector / FaceRef 临时对象。
std::size_t countFaceVertices(const std::string& line) {
    if (line.empty())
        return 0;
    const char* p = line.c_str() + 1;
    std::size_t count = 0;
    while (*p != '\0') {
        skipSpace(p);
        if (*p == '\0' || *p == '\r' || *p == '\n' || *p == '#')
            break;
        ++count;
        while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
            ++p;
        }
    }
    return count;
}

std::filesystem::path readDiffuseTextureFromMtl(const std::filesystem::path& mtlPath) {
    std::ifstream in(mtlPath, std::ios::binary);
    if (!in)
        return {};

    std::string line;
    while (std::getline(in, line)) {
        if (!startsWithToken(line, "map_Kd"))
            continue;
        std::istringstream ss(line.substr(6));
        std::string token;
        std::string last;
        while (ss >> token)
            last = token;
        if (!last.empty()) {
            return mtlPath.parent_path() / std::filesystem::u8path(last);
        }
    }
    return {};
}

bool checkedIntCount(std::size_t value) {
    return value <= static_cast<std::size_t>(std::numeric_limits<int>::max());
}

} // namespace

bool ObjModelLoader::load(const std::string& objFile, ObjModelData& out, std::string* message) {
    out = {};
    if (message)
        message->clear();

    const std::filesystem::path objPath = std::filesystem::u8path(objFile);

    // ------------------------------------------------------------------
    // Pass 1：流式统计。
    //
    // 旧实现会把整个 OBJ 拆成 vector<string>，再额外保存 vertexLines、
    // FaceJob、ParsedFace。对于数百万面 OBJ，这些容器是加载峰值的主要来源。
    // 新实现第一遍只保留几个计数器和一条 line，内存与文件大小基本无关。
    // ------------------------------------------------------------------
    std::ifstream first(objFile, std::ios::binary);
    if (!first) {
        if (message)
            *message = "无法打开 OBJ 文件: " + objFile;
        return false;
    }

    std::size_t vertexCount = 0;
    std::size_t uvCount = 0;
    std::size_t normalCount = 0;
    std::size_t triangleCount = 0;
    std::filesystem::path mtlPath;

    std::string line;
    while (std::getline(first, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        if (startsWithToken(line, "v")) {
            ++vertexCount;
        } else if (startsWithToken(line, "vt")) {
            ++uvCount;
        } else if (startsWithToken(line, "vn")) {
            ++normalCount;
        } else if (startsWithToken(line, "f")) {
            const std::size_t n = countFaceVertices(line);
            if (n >= 3u) {
                const std::size_t add = n - 2u;
                if (triangleCount > (std::numeric_limits<std::size_t>::max)() - add) {
                    if (message)
                        *message = "OBJ 三角形数量溢出";
                    return false;
                }
                triangleCount += add;
            }
        } else if (startsWithToken(line, "mtllib")) {
            std::string name = line.substr(6);
            const auto firstPos = name.find_first_not_of(" \t");
            const auto lastPos = name.find_last_not_of(" \t");
            if (firstPos != std::string::npos) {
                name = name.substr(firstPos, lastPos - firstPos + 1);
                mtlPath = objPath.parent_path() / std::filesystem::u8path(name);
            }
        }
    }
    first.close();

    if (vertexCount == 0u) {
        if (message)
            *message = "OBJ 中没有读取到任何 v 顶点";
        return false;
    }
    if (!checkedIntCount(vertexCount) || !checkedIntCount(uvCount) || !checkedIntCount(normalCount)) {
        if (message)
            *message = "OBJ v/vt/vn 数量超过当前 32 位索引实现上限";
        return false;
    }
    if (triangleCount > (std::numeric_limits<std::size_t>::max)() / 3u) {
        if (message)
            *message = "OBJ EBO 大小溢出";
        return false;
    }

    // 最终数据一次性定长分配；不经过 push_back 扩容和临时完整副本。
    PointCloud::Container points(vertexCount);
    out.mesh.triangleIndices.resize(triangleCount * 3u);

    std::vector<Vec2f> uvs;
    if (uvCount != 0u) {
        uvs.resize(uvCount);
        out.appearance.vertexUv.assign(vertexCount, {});
        out.appearance.hasUv.assign(vertexCount, 0u);
    }

    std::vector<Vec3f> normals;
    std::vector<Vec3f> normalSum;
    std::vector<std::uint32_t> vertexNormalCount;
    if (normalCount != 0u) {
        normals.resize(normalCount);
        out.appearance.vertexNormals.assign(vertexCount, {});
        normalSum.assign(vertexCount, {});
        vertexNormalCount.assign(vertexCount, 0u);
    }

    // ------------------------------------------------------------------
    // Pass 2：再次顺序读取，直接写最终 PointCloud / EBO / Appearance。
    // 每个 face 只使用一个可复用的小 vector，不保存所有 ParsedFace。
    // ------------------------------------------------------------------
    std::ifstream second(objFile, std::ios::binary);
    if (!second) {
        if (message)
            *message = "无法再次打开 OBJ 文件: " + objFile;
        out = {};
        return false;
    }

    std::size_t viWrite = 0;
    std::size_t vtWrite = 0;
    std::size_t vnWrite = 0;
    std::size_t indexWrite = 0;

    int seenV = 0;
    int seenT = 0;
    int seenN = 0;

    std::vector<FaceRef> faceRefs;
    faceRefs.reserve(8u);

    while (std::getline(second, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        if (startsWithToken(line, "v")) {
            if (viWrite >= points.size()) {
                if (message)
                    *message = "OBJ 顶点数量在两遍扫描间不一致";
                out = {};
                return false;
            }

            const char* p = line.c_str() + 1;
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (!parseFloatFast(p, x) || !parseFloatFast(p, y) || !parseFloatFast(p, z)) {
                if (message) {
                    *message = "OBJ 顶点解析失败，顶点序号=" + std::to_string(viWrite);
                }
                out = {};
                return false;
            }

            float r = 255.0f, g = 255.0f, b = 255.0f;
            const char* color = p;
            if (!parseFloatFast(color, r) || !parseFloatFast(color, g) || !parseFloatFast(color, b)) {
                r = g = b = 255.0f;
            }

            points[viWrite] = {
                {x, y, z}, packRgba(objColorToByte(r), objColorToByte(g), objColorToByte(b)), PointValid, {}};
            ++viWrite;
            ++seenV;
            continue;
        }

        if (startsWithToken(line, "vt")) {
            if (vtWrite < uvs.size()) {
                const char* p = line.c_str() + 2;
                parseFloatFast(p, uvs[vtWrite].x);
                parseFloatFast(p, uvs[vtWrite].y);
            }
            ++vtWrite;
            ++seenT;
            continue;
        }

        if (startsWithToken(line, "vn")) {
            if (vnWrite < normals.size()) {
                const char* p = line.c_str() + 2;
                Vec3f n{};
                if (parseFloatFast(p, n.x) && parseFloatFast(p, n.y) && parseFloatFast(p, n.z)) {
                    normals[vnWrite] = normalizeVec(n);
                }
            }
            ++vnWrite;
            ++seenN;
            continue;
        }

        if (!startsWithToken(line, "f"))
            continue;

        faceRefs.clear();
        const char* p = line.c_str() + 1;
        bool validFace = true;
        while (*p != '\0') {
            skipSpace(p);
            if (*p == '\0' || *p == '\r' || *p == '\n' || *p == '#')
                break;

            const FaceRef ref = parseFaceToken(p, seenV, seenT, seenN);
            if (ref.v < 0 || ref.v >= seenV) {
                validFace = false;
                break;
            }
            faceRefs.push_back(ref);

            while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
                ++p;
            }
        }

        if (!validFace || faceRefs.size() < 3u)
            continue;

        // 同一遍 face 解析同时完成 UV/Normal 归并，不再保留 ParsedFace。
        for (const FaceRef& ref : faceRefs) {
            const std::size_t dstVertex = static_cast<std::size_t>(ref.v);
            if (ref.vt >= 0 && static_cast<std::size_t>(ref.vt) < uvs.size() && !out.appearance.hasUv.empty() &&
                out.appearance.hasUv[dstVertex] == 0u) {
                out.appearance.vertexUv[dstVertex] = uvs[static_cast<std::size_t>(ref.vt)];
                out.appearance.hasUv[dstVertex] = 1u;
            }

            if (ref.vn >= 0 && static_cast<std::size_t>(ref.vn) < normals.size() && !normalSum.empty()) {
                const Vec3f& n = normals[static_cast<std::size_t>(ref.vn)];
                normalSum[dstVertex].x += n.x;
                normalSum[dstVertex].y += n.y;
                normalSum[dstVertex].z += n.z;
                ++vertexNormalCount[dstVertex];
            }
        }

        for (std::size_t k = 1u; k + 1u < faceRefs.size(); ++k) {
            if (indexWrite + 3u > out.mesh.triangleIndices.size()) {
                if (message)
                    *message = "OBJ 三角形数量在两遍扫描间不一致";
                out = {};
                return false;
            }
            out.mesh.triangleIndices[indexWrite++] = static_cast<std::uint32_t>(faceRefs[0].v);
            out.mesh.triangleIndices[indexWrite++] = static_cast<std::uint32_t>(faceRefs[k].v);
            out.mesh.triangleIndices[indexWrite++] = static_cast<std::uint32_t>(faceRefs[k + 1u].v);
        }
    }
    second.close();

    // 某些非法 face 会被忽略，因此第一遍的理论三角数可能大于实际写入数。
    // resize 只缩短逻辑大小，不制造第二份 EBO。
    if (indexWrite != out.mesh.triangleIndices.size()) {
        out.mesh.triangleIndices.resize(indexWrite);
        out.mesh.triangleIndices.shrink_to_fit();
    }

    // UV 原始表此后不再需要，先释放，再做法向最终归一化，降低阶段重叠峰值。
    std::vector<Vec2f>().swap(uvs);
    std::vector<FaceRef>().swap(faceRefs);

    if (!normalSum.empty()) {
#ifdef JMENGINE_USE_OPENMP
#pragma omp parallel for schedule(static) num_threads(processing::processingThreadCount())
#endif
        for (std::int64_t ii = 0; ii < static_cast<std::int64_t>(points.size()); ++ii) {
            const std::size_t i = static_cast<std::size_t>(ii);
            if (vertexNormalCount[i] == 0u)
                continue;
            const Vec3f n = normalizeVec(normalSum[i]);
            out.appearance.vertexNormals[i] = n;
            points[i].normal = n;
        }
    }

    // 所有原始 vn 和累计临时数组立即释放；完成后的常驻只保留最终模型数据。
    std::vector<Vec3f>().swap(normals);
    std::vector<Vec3f>().swap(normalSum);
    std::vector<std::uint32_t>().swap(vertexNormalCount);

    if (!mtlPath.empty()) {
        const auto tex = readDiffuseTextureFromMtl(mtlPath);
        if (!tex.empty())
            out.appearance.diffuseTexturePath = tex.u8string();
    }

    out.cloud = std::make_shared<PointCloud>(std::move(points));

    if (message) {
        std::ostringstream ss;
        ss << "OBJ 加载完成：顶点=" << out.cloud->size() << "，三角形=" << out.mesh.triangleCount()
           << "，低峰值双遍流式解析";
#ifdef JMENGINE_USE_OPENMP
        ss << "，OpenMP=" << processing::processingThreadCount() << "线程(法向归一化，保留1核)";
#else
        ss << "，OpenMP=关闭";
#endif
        if (!out.appearance.diffuseTexturePath.empty())
            ss << "，检测到纹理";
        *message = ss.str();
    }
    return true;
}

bool ObjAppearanceData::hasTextureCoordinates() const noexcept {
    return std::find(hasUv.begin(), hasUv.end(), static_cast<std::uint8_t>(1u)) != hasUv.end();
}

bool ObjAppearanceData::hasNormals() const noexcept {
    for (const auto& n : vertexNormals) {
        if (n.x != 0.0f || n.y != 0.0f || n.z != 0.0f)
            return true;
    }
    return false;
}

} // namespace JMEngine
