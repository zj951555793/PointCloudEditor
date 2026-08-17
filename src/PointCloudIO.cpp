#include <JMEngine/PointCloudIO.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace JMEngine {
namespace {

// -----------------------------
// 通用小工具
// -----------------------------

// 统一写错误信息，调用者没有传 errorMessage 时自动忽略。
void setError(std::string* dst, const std::string& text) {
    if (dst)
        *dst = text;
}

// 转小写，用于扩展名、PLY property/type 等大小写无关比较。
std::string lowerCopy(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

// 删除行尾可能残留的 '\r'，兼容 Windows CRLF 文件在 Linux 下读取。
void trimTrailingCr(std::string& text) {
    if (!text.empty() && text.back() == '\r')
        text.pop_back();
}

// 将 0~255 的 RGBA 打包到 uint32_t。
// 当前库约定：最低字节 R，接着 G/B/A。
std::uint32_t packRgba(int r, int g, int b, int a = 255) {
    auto clampByte = [](int v) -> std::uint32_t { return static_cast<std::uint32_t>(std::clamp(v, 0, 255)); };
    return clampByte(r) | (clampByte(g) << 8u) | (clampByte(b) << 16u) | (clampByte(a) << 24u);
}

// OBJ 仅点云加载时使用的轻量辅助：不构造 Mesh/Appearance。
// PointCloudIO::loadObj() 的调用方只需要 v 顶点，因此没有必要先构造完整
// ObjModelData 再把 mesh/uv/normal 丢掉。
bool isObjVertexLine(const std::string& line) {
    return !line.empty() && line[0] == 'v' && line.size() > 1u &&
           std::isspace(static_cast<unsigned char>(line[1])) != 0;
}

inline void skipObjSpace(const char*& p) {
    while (*p == ' ' || *p == '\t')
        ++p;
}

bool parseObjFloat(const char*& p, float& out) {
    skipObjSpace(p);
    if (*p == '\0' || *p == '\r' || *p == '\n')
        return false;
    char* end = nullptr;
    out = std::strtof(p, &end);
    if (end == p)
        return false;
    p = end;
    return true;
}

std::uint8_t objColorByte(float value) {
    if (value >= 0.0f && value <= 1.0f)
        value *= 255.0f;
    value = std::clamp(value, 0.0f, 255.0f);
    return static_cast<std::uint8_t>(value + 0.5f);
}

// -----------------------------
// PLY 类型解析
// -----------------------------

enum class PlyScalarType { Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64, Unknown };

struct PlyProperty {
    PlyScalarType type{PlyScalarType::Unknown};
    std::string name;
};

PlyScalarType parsePlyType(const std::string& typeName) {
    const std::string t = lowerCopy(typeName);
    if (t == "char" || t == "int8")
        return PlyScalarType::Int8;
    if (t == "uchar" || t == "uint8")
        return PlyScalarType::UInt8;
    if (t == "short" || t == "int16")
        return PlyScalarType::Int16;
    if (t == "ushort" || t == "uint16")
        return PlyScalarType::UInt16;
    if (t == "int" || t == "int32")
        return PlyScalarType::Int32;
    if (t == "uint" || t == "uint32")
        return PlyScalarType::UInt32;
    if (t == "float" || t == "float32")
        return PlyScalarType::Float32;
    if (t == "double" || t == "float64")
        return PlyScalarType::Float64;
    return PlyScalarType::Unknown;
}

// 判断当前 CPU 是否为小端。
// Windows x64、x86、ARM64 RK3588 都是小端，但这里仍做通用处理。
bool hostIsLittleEndian() {
    const std::uint16_t value = 1;
    return *reinterpret_cast<const std::uint8_t*>(&value) == 1;
}

template <typename T> bool readLittleEndian(std::istream& in, T& value) {
    std::array<unsigned char, sizeof(T)> bytes{};
    if (!in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()))) {
        return false;
    }
    if (!hostIsLittleEndian())
        std::reverse(bytes.begin(), bytes.end());
    std::memcpy(&value, bytes.data(), sizeof(T));
    return true;
}

// 从 binary_little_endian PLY 中读取一个标量，并统一转换为 double。
bool readPlyBinaryScalar(std::istream& in, PlyScalarType type, double& out) {
    switch (type) {
    case PlyScalarType::Int8: {
        std::int8_t v{};
        if (!in.read(reinterpret_cast<char*>(&v), sizeof(v)))
            return false;
        out = static_cast<double>(v);
        return true;
    }
    case PlyScalarType::UInt8: {
        std::uint8_t v{};
        if (!in.read(reinterpret_cast<char*>(&v), sizeof(v)))
            return false;
        out = static_cast<double>(v);
        return true;
    }
    case PlyScalarType::Int16: {
        std::int16_t v{};
        if (!readLittleEndian(in, v))
            return false;
        out = static_cast<double>(v);
        return true;
    }
    case PlyScalarType::UInt16: {
        std::uint16_t v{};
        if (!readLittleEndian(in, v))
            return false;
        out = static_cast<double>(v);
        return true;
    }
    case PlyScalarType::Int32: {
        std::int32_t v{};
        if (!readLittleEndian(in, v))
            return false;
        out = static_cast<double>(v);
        return true;
    }
    case PlyScalarType::UInt32: {
        std::uint32_t v{};
        if (!readLittleEndian(in, v))
            return false;
        out = static_cast<double>(v);
        return true;
    }
    case PlyScalarType::Float32: {
        float v{};
        if (!readLittleEndian(in, v))
            return false;
        out = static_cast<double>(v);
        return true;
    }
    case PlyScalarType::Float64: {
        double v{};
        if (!readLittleEndian(in, v))
            return false;
        out = v;
        return true;
    }
    default:
        return false;
    }
}

} // namespace

// -----------------------------
// 自动格式识别
// -----------------------------

std::shared_ptr<PointCloud> PointCloudIO::load(const std::string& fileName, std::string* errorMessage) {
    if (errorMessage)
        errorMessage->clear();

    const auto dot = fileName.find_last_of('.');
    if (dot == std::string::npos) {
        setError(errorMessage, "文件没有扩展名，无法判断 OBJ/PLY 格式");
        return nullptr;
    }

    const std::string ext = lowerCopy(fileName.substr(dot + 1));
    if (ext == "obj")
        return loadObj(fileName, errorMessage);
    if (ext == "ply")
        return loadPly(fileName, errorMessage);

    setError(errorMessage, "当前仅支持 .obj 和 .ply 文件");
    return nullptr;
}

// -----------------------------
// OBJ
// -----------------------------

std::shared_ptr<PointCloud> PointCloudIO::loadObj(const std::string& fileName, std::string* errorMessage) {
    if (errorMessage)
        errorMessage->clear();

    // 低峰值两遍流式读取。这里明确只读取 v 顶点；不会构造 ObjMeshData、
    // ObjAppearanceData、face 临时数组，因此作为“点云方式打开 OBJ”时的
    // 峰值接近最终 PointCloud 本身。
    std::ifstream first(fileName, std::ios::binary);
    if (!first) {
        setError(errorMessage, "无法打开 OBJ 文件: " + fileName);
        return nullptr;
    }

    std::size_t vertexCount = 0;
    std::string line;
    while (std::getline(first, line)) {
        if (isObjVertexLine(line))
            ++vertexCount;
    }
    first.close();

    if (vertexCount == 0u) {
        setError(errorMessage, "OBJ 中没有读取到任何 v 顶点");
        return nullptr;
    }

    PointCloud::Container points(vertexCount);
    std::ifstream second(fileName, std::ios::binary);
    if (!second) {
        setError(errorMessage, "无法再次打开 OBJ 文件: " + fileName);
        return nullptr;
    }

    std::size_t dst = 0;
    while (std::getline(second, line)) {
        if (!isObjVertexLine(line))
            continue;
        if (dst >= points.size()) {
            setError(errorMessage, "OBJ 顶点数量在两遍扫描间不一致");
            return nullptr;
        }

        const char* p = line.c_str() + 1;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        if (!parseObjFloat(p, x) || !parseObjFloat(p, y) || !parseObjFloat(p, z)) {
            setError(errorMessage, "OBJ 顶点解析失败，顶点序号=" + std::to_string(dst));
            return nullptr;
        }

        float r = 255.0f, g = 255.0f, b = 255.0f;
        const char* color = p;
        if (!parseObjFloat(color, r) || !parseObjFloat(color, g) || !parseObjFloat(color, b)) {
            r = g = b = 255.0f;
        }

        points[dst++] = {{x, y, z}, packRgba(objColorByte(r), objColorByte(g), objColorByte(b)), PointValid, {}};
    }

    if (dst != points.size()) {
        setError(errorMessage, "OBJ 顶点数量在两遍扫描间不一致");
        return nullptr;
    }

    return std::make_shared<PointCloud>(std::move(points));
}

// -----------------------------
// PLY
// -----------------------------

std::shared_ptr<PointCloud> PointCloudIO::loadPly(const std::string& fileName, std::string* errorMessage) {
    if (errorMessage)
        errorMessage->clear();

    // 使用 binary 打开：ASCII PLY 仍可 getline，而 binary_little_endian 也不会被文本模式改写字节。
    std::ifstream in(fileName, std::ios::binary);
    if (!in) {
        setError(errorMessage, "无法打开 PLY 文件: " + fileName);
        return nullptr;
    }

    std::string line;
    if (!std::getline(in, line)) {
        setError(errorMessage, "PLY 文件为空");
        return nullptr;
    }
    trimTrailingCr(line);
    if (lowerCopy(line) != "ply") {
        setError(errorMessage, "不是有效的 PLY 文件");
        return nullptr;
    }

    enum class PlyFormat { Unknown, Ascii, BinaryLittleEndian };
    PlyFormat format = PlyFormat::Unknown;
    bool inVertexElement = false;
    std::size_t vertexCount = 0;
    std::vector<PlyProperty> vertexProperties;
    bool headerEnded = false;

    // PLY header 可以包含多个 element；这里只记录 vertex 元素相关属性。
    while (std::getline(in, line)) {
        trimTrailingCr(line);
        std::istringstream ss(line);
        std::string key;
        ss >> key;
        key = lowerCopy(key);

        if (key == "format") {
            std::string formatName;
            ss >> formatName;
            formatName = lowerCopy(formatName);
            if (formatName == "ascii")
                format = PlyFormat::Ascii;
            else if (formatName == "binary_little_endian")
                format = PlyFormat::BinaryLittleEndian;
            else {
                setError(errorMessage, "暂不支持该 PLY format: " + formatName);
                return nullptr;
            }
        } else if (key == "element") {
            std::string elementName;
            std::size_t count = 0;
            ss >> elementName >> count;
            elementName = lowerCopy(elementName);
            inVertexElement = (elementName == "vertex");
            if (inVertexElement)
                vertexCount = count;
        } else if (key == "property" && inVertexElement) {
            std::string typeName;
            ss >> typeName;

            // vertex 里出现 list property 的情况非常少见，并且需要额外处理变长数据。
            // 当前轻量点云格式只支持标量 vertex property，明确报错比静默读错更安全。
            if (lowerCopy(typeName) == "list") {
                setError(errorMessage, "暂不支持 vertex list property 的 PLY");
                return nullptr;
            }

            std::string propertyName;
            ss >> propertyName;
            const PlyScalarType type = parsePlyType(typeName);
            if (type == PlyScalarType::Unknown) {
                setError(errorMessage, "不支持的 PLY property 类型: " + typeName);
                return nullptr;
            }
            vertexProperties.push_back({type, lowerCopy(propertyName)});
        } else if (key == "end_header") {
            headerEnded = true;
            break;
        }
    }

    if (!headerEnded) {
        setError(errorMessage, "PLY header 缺少 end_header");
        return nullptr;
    }
    if (format == PlyFormat::Unknown) {
        setError(errorMessage, "PLY header 缺少 format");
        return nullptr;
    }
    if (vertexCount == 0) {
        setError(errorMessage, "PLY header 中 vertex 数量为 0");
        return nullptr;
    }
    if (vertexProperties.empty()) {
        setError(errorMessage, "PLY vertex 没有 property");
        return nullptr;
    }

    auto findProperty = [&](const char* name) -> int {
        for (std::size_t i = 0; i < vertexProperties.size(); ++i) {
            if (vertexProperties[i].name == name)
                return static_cast<int>(i);
        }
        return -1;
    };

    // 某些扫描软件会把颜色命名成 r/g/b 或 diffuse_red/diffuse_green/diffuse_blue。
    // 这里按常见别名逐个回退，避免“文件明明有 RGB，但显示成整片白色”。
    auto findAnyProperty = [&](std::initializer_list<const char*> names) -> int {
        for (const char* name : names) {
            const int index = findProperty(name);
            if (index >= 0)
                return index;
        }
        return -1;
    };

    const int ix = findProperty("x");
    const int iy = findProperty("y");
    const int iz = findProperty("z");
    const int inx = findAnyProperty({"nx", "normal_x"});
    const int iny = findAnyProperty({"ny", "normal_y"});
    const int inz = findAnyProperty({"nz", "normal_z"});
    const int ir = findAnyProperty({"red", "r", "diffuse_red"});
    const int ig = findAnyProperty({"green", "g", "diffuse_green"});
    const int ib = findAnyProperty({"blue", "b", "diffuse_blue"});
    const int ia = findAnyProperty({"alpha", "a", "diffuse_alpha"});

    if (ix < 0 || iy < 0 || iz < 0) {
        setError(errorMessage, "PLY vertex 必须包含 x/y/z 属性");
        return nullptr;
    }

    // vertexCount 已由 header 精确给出，直接 resize 到最终大小。
    // 这样 PLY 加载期间只有一份最终 Point 数组，不经过 push_back/扩容阶段。
    PointCloud::Container points(vertexCount);
    std::vector<double> values(vertexProperties.size(), 0.0);

    for (std::size_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        bool ok = true;

        if (format == PlyFormat::Ascii) {
            if (!std::getline(in, line)) {
                ok = false;
            } else {
                trimTrailingCr(line);
                // 避免每个顶点构造 std::istringstream 及其内部缓冲。
                // strtod 直接在当前 line 上前进，只保留一个复用字符串。
                const char* p = line.c_str();
                for (double& value : values) {
                    while (*p == ' ' || *p == '\t')
                        ++p;
                    if (*p == '\0') {
                        ok = false;
                        break;
                    }
                    char* end = nullptr;
                    value = std::strtod(p, &end);
                    if (end == p) {
                        ok = false;
                        break;
                    }
                    p = end;
                }
            }
        } else {
            // binary_little_endian：严格按照 header 中 property 顺序逐项读取。
            for (std::size_t propertyIndex = 0; propertyIndex < vertexProperties.size(); ++propertyIndex) {
                if (!readPlyBinaryScalar(in, vertexProperties[propertyIndex].type, values[propertyIndex])) {
                    ok = false;
                    break;
                }
            }
        }

        if (!ok) {
            setError(errorMessage, "PLY 顶点数据解析失败，顶点序号: " + std::to_string(vertexIndex));
            return nullptr;
        }

        const int r = ir >= 0 ? static_cast<int>(values[static_cast<std::size_t>(ir)]) : 255;
        const int g = ig >= 0 ? static_cast<int>(values[static_cast<std::size_t>(ig)]) : 255;
        const int b = ib >= 0 ? static_cast<int>(values[static_cast<std::size_t>(ib)]) : 255;
        const int a = ia >= 0 ? static_cast<int>(values[static_cast<std::size_t>(ia)]) : 255;

        Point& point = points[vertexIndex];
        point.position = {static_cast<float>(values[static_cast<std::size_t>(ix)]),
                          static_cast<float>(values[static_cast<std::size_t>(iy)]),
                          static_cast<float>(values[static_cast<std::size_t>(iz)])};
        point.rgba = packRgba(r, g, b, a);
        point.flags = PointValid;
        if (inx >= 0 && iny >= 0 && inz >= 0) {
            point.normal = {static_cast<float>(values[static_cast<std::size_t>(inx)]),
                            static_cast<float>(values[static_cast<std::size_t>(iny)]),
                            static_cast<float>(values[static_cast<std::size_t>(inz)])};
        }
    }

    return std::make_shared<PointCloud>(std::move(points));
}

// -----------------------------
// PLY 导出
// -----------------------------

bool PointCloudIO::savePly(const PointCloud& cloud, const std::string& fileName, std::string* errorMessage) {
    if (errorMessage)
        errorMessage->clear();

    std::ofstream out(fileName);
    if (!out) {
        setError(errorMessage, "无法创建 PLY 文件: " + fileName);
        return false;
    }

    // 导出只写未删除点；软删除点不会出现在结果文件里。
    out << "ply\n"
        << "format ascii 1.0\n"
        << "comment generated by JMEngine\n"
        << "element vertex " << cloud.activeCount() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property uchar red\n"
        << "property uchar green\n"
        << "property uchar blue\n"
        << "property uchar alpha\n"
        << "end_header\n";

    for (const auto& p : cloud.points()) {
        if ((p.flags & PointDeleted) != 0)
            continue;

        const auto rgba = p.rgba;
        const int r = static_cast<int>(rgba & 0xffu);
        const int g = static_cast<int>((rgba >> 8u) & 0xffu);
        const int b = static_cast<int>((rgba >> 16u) & 0xffu);
        const int a = static_cast<int>((rgba >> 24u) & 0xffu);

        out << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' ' << r << ' ' << g << ' ' << b << ' '
            << a << '\n';
    }

    if (!out.good()) {
        setError(errorMessage, "写入 PLY 文件失败");
        return false;
    }
    return true;
}

} // namespace JMEngine
