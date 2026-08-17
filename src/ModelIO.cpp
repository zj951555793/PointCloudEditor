#include <JMEngine/ModelIO.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace JMEngine {
namespace {
std::string lowerExt(const std::string& name) {
    std::string e = std::filesystem::path(name).extension().string();
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return e;
}
void setError(std::string* out, const std::string& value) {
    if (out)
        *out = value;
}
bool finite3(float x, float y, float z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}
std::array<unsigned, 4> rgba(std::uint32_t c) {
    return {c & 0xffu, (c >> 8u) & 0xffu, (c >> 16u) & 0xffu, (c >> 24u) & 0xffu};
}
} // namespace

std::shared_ptr<PointCloud> loadTextCloud(const std::string& fileName, const char* formatName,
                                          std::string* errorMessage) {
    std::ifstream in(fileName, std::ios::binary);
    if (!in) {
        setError(errorMessage, std::string("无法打开 ") + formatName + " 文件");
        return {};
    }

    PointCloud::Container points;
    points.reserve(1u << 16u);
    std::string line;
    std::size_t skipped = 0;
    std::size_t ignoredIntensity = 0;

    while (std::getline(in, line)) {
        if (line.empty())
            continue;
        for (char& c : line)
            if (c == ',' || c == ';' || c == '\t')
                c = ' ';

        const auto first = line.find_first_not_of(" \r\n");
        if (first == std::string::npos || line[first] == '#' || line[first] == '/' || line[first] == '%')
            continue;

        // 不为每行构造 vector<double>，减少千万点 ASC/TXT 导入时的小对象分配。
        std::istringstream ss(line);
        double v[12]{};
        int n = 0;
        while (n < 12 && (ss >> v[n]))
            ++n;
        if (n < 3) {
            ++skipped;
            continue;
        }

        Point p{};
        p.position = {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
        if (!finite3(p.position.x, p.position.y, p.position.z)) {
            ++skipped;
            continue;
        }
        p.flags = PointValid;
        p.rgba = 0xffffffffu;

        // 工业 ASC 常见的 4 列是 XYZI。当前 Point 没有 intensity 字段，明确忽略，
        // 避免把强度误解析成红色通道。
        if (n == 4)
            ++ignoredIntensity;

        // 6+ 列按 XYZRGB[Normal]。RGB 同时兼容 0..1 和 0..255。
        if (n >= 6) {
            const auto cv = [](double a) -> unsigned {
                if (a >= 0.0 && a <= 1.0)
                    a *= 255.0;
                return static_cast<unsigned>(std::clamp(a, 0.0, 255.0) + 0.5);
            };
            const auto r = cv(v[3]), g = cv(v[4]), b = cv(v[5]);
            p.rgba = r | (g << 8u) | (b << 16u) | 0xff000000u;
        }

        if (n >= 9) {
            p.normal = {static_cast<float>(v[6]), static_cast<float>(v[7]), static_cast<float>(v[8])};
            const float l2 = p.normal.x * p.normal.x + p.normal.y * p.normal.y + p.normal.z * p.normal.z;
            if (l2 > 1e-24f && std::isfinite(l2)) {
                const float inv = 1.0f / std::sqrt(l2);
                p.normal.x *= inv;
                p.normal.y *= inv;
                p.normal.z *= inv;
            } else {
                p.normal = {};
            }
        }
        points.push_back(p);
    }

    if (points.empty()) {
        setError(errorMessage, std::string(formatName) + " 中没有识别到有效 XYZ 点");
        return {};
    }

    if (errorMessage) {
        *errorMessage = "已读取 " + std::to_string(points.size()) + " 点，跳过 " + std::to_string(skipped) + " 行";
        if (ignoredIntensity > 0)
            *errorMessage += "；检测到 XYZI=" + std::to_string(ignoredIntensity) + " 点（intensity 已忽略）";
    }
    return std::make_shared<PointCloud>(std::move(points));
}

std::shared_ptr<PointCloud> ModelIO::loadTxt(const std::string& fileName, std::string* errorMessage) {
    return loadTextCloud(fileName, "TXT", errorMessage);
}

std::shared_ptr<PointCloud> ModelIO::loadAsc(const std::string& fileName, std::string* errorMessage) {
    return loadTextCloud(fileName, "ASC", errorMessage);
}

bool ModelIO::save(const PointCloud& cloud, const TriangleMesh* mesh, const std::string& fileName,
                   std::string* errorMessage) {
    const auto ext = lowerExt(fileName);
    if (ext == ".obj")
        return mesh ? saveObj(cloud, *mesh, fileName, errorMessage)
                    : (setError(errorMessage, "OBJ 导出需要三角网格"), false);
    if (ext == ".stl")
        return mesh ? saveStl(cloud, *mesh, fileName, errorMessage)
                    : (setError(errorMessage, "STL 导出需要三角网格"), false);
    if (ext == ".ply")
        return savePly(cloud, mesh, fileName, errorMessage);
    if (ext == ".asc")
        return saveAsc(cloud, fileName, errorMessage);
    setError(errorMessage, "不支持的导出格式: " + ext);
    return false;
}

bool ModelIO::saveObj(const PointCloud& cloud, const TriangleMesh& mesh, const std::string& fileName,
                      std::string* errorMessage) {
    std::ofstream out(fileName);
    if (!out) {
        setError(errorMessage, "无法创建 OBJ 文件");
        return false;
    }
    out << std::setprecision(9);
    for (const auto& p : cloud.points())
        out << "v " << p.position.x << ' ' << p.position.y << ' ' << p.position.z << '\n';
    bool hasNormals = true;
    for (const auto& p : cloud.points()) {
        const float l2 = p.normal.x * p.normal.x + p.normal.y * p.normal.y + p.normal.z * p.normal.z;
        if (!(l2 > 1e-20f)) {
            hasNormals = false;
            break;
        }
    }
    if (hasNormals)
        for (const auto& p : cloud.points())
            out << "vn " << p.normal.x << ' ' << p.normal.y << ' ' << p.normal.z << '\n';
    const auto& idx = mesh.indices();
    for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
        if (!mesh.triangleActive(static_cast<TriangleId>(i / 3)))
            continue;
        const auto a = idx[i] + 1u, b = idx[i + 1] + 1u, c = idx[i + 2] + 1u;
        if (hasNormals)
            out << "f " << a << "//" << a << ' ' << b << "//" << b << ' ' << c << "//" << c << '\n';
        else
            out << "f " << a << ' ' << b << ' ' << c << '\n';
    }
    return static_cast<bool>(out);
}

bool ModelIO::saveStl(const PointCloud& cloud, const TriangleMesh& mesh, const std::string& fileName,
                      std::string* errorMessage) {
    std::ofstream out(fileName);
    if (!out) {
        setError(errorMessage, "无法创建 STL 文件");
        return false;
    }
    out << "solid JMEngine\n" << std::setprecision(9);
    const auto& pts = cloud.points();
    const auto& idx = mesh.indices();
    for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
        if (!mesh.triangleActive(static_cast<TriangleId>(i / 3)))
            continue;
        const auto a = idx[i], b = idx[i + 1], c = idx[i + 2];
        if (a >= pts.size() || b >= pts.size() || c >= pts.size())
            continue;
        const auto& p0 = pts[a].position;
        const auto& p1 = pts[b].position;
        const auto& p2 = pts[c].position;
        const float ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z, vx = p2.x - p0.x, vy = p2.y - p0.y,
                    vz = p2.z - p0.z;
        float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const float l = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (l > 1e-20f) {
            nx /= l;
            ny /= l;
            nz /= l;
        }
        out << "facet normal " << nx << ' ' << ny << ' ' << nz << "\n outer loop\n";
        out << "  vertex " << p0.x << ' ' << p0.y << ' ' << p0.z << "\n  vertex " << p1.x << ' ' << p1.y << ' ' << p1.z
            << "\n  vertex " << p2.x << ' ' << p2.y << ' ' << p2.z << "\n endloop\nendfacet\n";
    }
    out << "endsolid JMEngine\n";
    return static_cast<bool>(out);
}

bool ModelIO::savePly(const PointCloud& cloud, const TriangleMesh* mesh, const std::string& fileName,
                      std::string* errorMessage) {
    std::ofstream out(fileName);
    if (!out) {
        setError(errorMessage, "无法创建 PLY 文件");
        return false;
    }
    std::size_t faces = mesh ? mesh->activeTriangleCount() : 0;
    const std::size_t vertexCount = mesh ? cloud.size() : cloud.activeCount();
    out << "ply\nformat ascii 1.0\nelement vertex " << vertexCount
        << "\nproperty float x\nproperty float y\nproperty float z\n";
    out << "property float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar "
           "green\nproperty uchar blue\nproperty uchar alpha\n";
    if (mesh)
        out << "element face " << faces << "\nproperty list uchar int vertex_indices\n";
    out << "end_header\n" << std::setprecision(9);
    for (const auto& p : cloud.points()) {
        if (!mesh && (p.flags & PointDeleted) != 0)
            continue;
        auto c = rgba(p.rgba);
        out << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' ' << p.normal.x << ' ' << p.normal.y
            << ' ' << p.normal.z << ' ' << c[0] << ' ' << c[1] << ' ' << c[2] << ' ' << c[3] << '\n';
    }
    if (mesh) {
        const auto& idx = mesh->indices();
        for (std::size_t i = 0; i + 2 < idx.size(); i += 3)
            if (mesh->triangleActive(static_cast<TriangleId>(i / 3)))
                out << "3 " << idx[i] << ' ' << idx[i + 1] << ' ' << idx[i + 2] << '\n';
    }
    return static_cast<bool>(out);
}


bool ModelIO::saveAsc(const PointCloud& cloud, const std::string& fileName, std::string* errorMessage) {
    std::ofstream out(fileName, std::ios::binary);
    if (!out) {
        setError(errorMessage, "无法创建 ASC 文件");
        return false;
    }

    // 9 位有效数字足以 round-trip float；逐点流式写，不创建临时点数组。
    out << std::setprecision(9);
    std::size_t written = 0;
    for (const auto& p : cloud.points()) {
        if ((p.flags & PointDeleted) != 0 || !finite3(p.position.x, p.position.y, p.position.z))
            continue;
        const auto c = rgba(p.rgba);
        out << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
            << c[0] << ' ' << c[1] << ' ' << c[2] << '\n';
        if (!out) {
            setError(errorMessage, "写入 ASC 文件失败（磁盘空间或 I/O 错误）");
            return false;
        }
        ++written;
    }
    if (written == 0) {
        setError(errorMessage, "没有可导出的有效点");
        return false;
    }
    if (errorMessage)
        *errorMessage = "已导出 ASC：" + std::to_string(written) + " 点（XYZRGB）";
    return true;
}

} // namespace JMEngine
