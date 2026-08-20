#include <JMEngine/CpuSelector.h>
#include <JMEngine/CpuMeshSelector.h>
#include <JMEngine/MeshSelectionClosure.h>
#include <JMEngine/processing/Processing.h>
#include <JMEngine/processing/Parallel.h>
#include <JMEngine/processing/Diagnostics.h>
#include <JMEngine/ColorPicking24.h>
#include <JMEngine/BlockPicking24.h>
#include <JMEngine/PixelIdPicker.h>
#include <JMEngine/JMEngine.h>
#include <JMEngine/PointCloudIO.h>
#include <JMEngine/MeshUtils.h>
#include <JMEngine/ModelIO.h>
#include <JMEngine/ObjModelLoader.h>
#include <JMEngine/TriangleMesh.h>
#include <JMEngine/edit/MeshEditSession.h>
#include <JMEngine/JMScanner.h>

#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

using namespace JMEngine;

namespace {

// 构造一份小点云，用于验证编辑功能。
std::shared_ptr<PointCloud> makeCloud() {
    PointCloud::Container pts = {{{-0.5f, 0.0f, 0.0f}, 0xff0000ffu, PointValid},
                                 {{0.0f, 0.0f, 0.0f}, 0x00ff00ffu, PointValid},
                                 {{0.5f, 0.0f, 0.0f}, 0x0000ffffu, PointValid},
                                 {{2.0f, 0.0f, 0.0f}, 0xffffffffu, PointValid}};
    return std::make_shared<PointCloud>(std::move(pts));
}

void testSelectionDeleteUndoRedo() {
    auto cloud = makeCloud();
    Engine editor(cloud);

    // set() 会自动排序去重，所以重复 2 最终只保留一次。
    editor.select({1, 2, 2});
    assert(editor.selection().size() == 2);

    // 这里专门覆盖 std::set_difference + std::back_inserter 的代码路径，
    // 确保 Selection.h 已正确包含 <iterator>。
    editor.removeSelection({2});
    assert(editor.selection().size() == 1);
    assert(editor.selection().ids()[0] == 1);

    assert(editor.deleteSelection());
    assert(editor.lastChangeKind() == ChangeKind::Flags);
    assert(editor.lastChangedIds() == std::vector<PointId>{1});
    assert(cloud->activeCount() == 3);

    assert(editor.undo());
    assert(cloud->activeCount() == 4);

    assert(editor.redo());
    assert(cloud->activeCount() == 3);
}

void testCrop() {
    auto cloud = makeCloud();
    Engine editor(cloud);

    assert(editor.crop({{-1.f, -1.f, -1.f}, {1.f, 1.f, 1.f}}));
    assert(cloud->activeCount() == 3);
    assert(editor.undo());
    assert(cloud->activeCount() == 4);
}

void testTransform() {
    auto cloud = makeCloud();
    Engine editor(cloud);

    editor.select({0});
    Mat4f translate = Mat4f::identity();
    translate.m[12] = 1.0f; // OpenGL 列主序：X 平移。

    assert(editor.transform(translate));
    assert(cloud->at(0).position.x == 0.5f);

    assert(editor.undo());
    assert(cloud->at(0).position.x == -0.5f);
}

void testReplaceCloudClearsEditState() {
    Engine editor(makeCloud());
    editor.select({1});
    assert(editor.deleteSelection());
    assert(editor.canUndo());
    editor.setPointCloud(makeCloud());
    assert(editor.selection().empty());
    assert(!editor.canUndo());
    assert(!editor.canRedo());
}

void testCpuSelector() {
    auto cloud = makeCloud();
    const Mat4f mvp = Mat4f::identity();

    const auto ids = CpuSelector::rectangle(*cloud, mvp, {100, 100}, {20, 40, 80, 60});
    assert(!ids.empty());
}

void testPixelIdPicker() {
    // 模拟一张 4x4 的 GPU GL_R32UI PointId 纹理。
    // 同一个点覆盖多个像素，因此 PixelIdPicker 必须过滤背景并去重。
    constexpr int w = 4;
    constexpr int h = 4;
    const std::uint32_t fb[w * h] = {kInvalidPointId, 1, 1, kInvalidPointId, 2, 2, 3, 3, 2, 4, 4, 3, kInvalidPointId, 4,
                                     kInvalidPointId, 5};

    PixelIdPicker picker(
        w, h,
        [&](int x, int y, int rw, int rh, std::uint32_t* dst) {
            for (int row = 0; row < rh; ++row) {
                for (int col = 0; col < rw; ++col) {
                    dst[row * rw + col] = fb[(y + row) * w + (x + col)];
                }
            }
            return true;
        },
        false);

    const auto ids = picker.pickRectangle(0, 0, 3, 3);
    assert((ids == std::vector<PointId>{1, 2, 3, 4, 5}));

    // 继续覆盖新增的圆形 / Lasso / Brush GPU 形状过滤逻辑。
    const auto circle = picker.pickCircle(1, 1, 1);
    assert(!circle.empty());

    const std::vector<Point2i> polygon = {{0, 0}, {3, 0}, {3, 3}, {0, 3}};
    const auto lasso = picker.pickLasso(polygon);
    assert(!lasso.empty());

    const std::vector<Point2i> brushPath = {{0, 1}, {3, 1}};
    const auto brush = picker.pickBrushStroke(brushPath, 1);
    assert(!brush.empty());

    // 再验证 Qt 常用的“输入左上原点 + glReadPixels 左下原点”转换。
    // GL framebuffer 内存仍按左下原点保存；点击 UI 左上角 (1,0) 应命中 GL 最后一行的 id=4。
    PixelIdPicker topLeftPicker(
        w, h,
        [&](int x, int y, int rw, int rh, std::uint32_t* dst) {
            for (int row = 0; row < rh; ++row) {
                for (int col = 0; col < rw; ++col) {
                    dst[row * rw + col] = fb[(y + row) * w + (x + col)];
                }
            }
            return true;
        },
        true);
    assert(topLeftPicker.pickPoint(1, 0) == 4);
}

void testPointCloudIo() {
    const std::filesystem::path dataDir = JMENGINE_TEST_DATA_DIR;

    std::string error;
    auto ply = PointCloudIO::load((dataDir / "sample_ascii.ply").string(), &error);
    assert(ply && error.empty());
    assert(ply->size() == 4);

    error.clear();
    auto obj = PointCloudIO::load((dataDir / "sample.obj").string(), &error);
    assert(obj && error.empty());
    assert(obj->size() == 4);

    // OBJ 一次性模型加载器：验证点、网格拓扑都来自 Core，且无需 Qt。
    ObjModelData objModel;
    std::string objMessage;
    assert(ObjModelLoader::load((dataDir / "sample.obj").string(), objModel, &objMessage));
    assert(objModel.cloud && objModel.cloud->size() == 4);
    assert(objModel.mesh.triangleCount() == 1);

    // 工业扫描软件经常输出 binary_little_endian PLY，这里也必须覆盖。
    error.clear();
    auto binaryPly = PointCloudIO::load((dataDir / "sample_binary_le.ply").string(), &error);
    assert(binaryPly && error.empty());
    assert(binaryPly->size() == 4);

    // ASC：兼容空格/逗号/分号/Tab，以及 0..255 / 0..1 RGB。
    error.clear();
    auto asc = ModelIO::loadAsc((dataDir / "sample.asc").string(), &error);
    assert(asc);
    assert(asc->size() == 4);

    const auto ascOutput = std::filesystem::temp_directory_path() / "JMEngine_io_test.asc";
    error.clear();
    assert(ModelIO::saveAsc(*asc, ascOutput.string(), &error));
    auto ascReload = ModelIO::loadAsc(ascOutput.string(), &error);
    assert(ascReload && ascReload->size() == asc->size());

    // 验证编辑结果能够重新保存成 PLY。
    Engine editor(obj);
    editor.select({1});
    assert(editor.deleteSelection());

    const auto output = std::filesystem::temp_directory_path() / "JMEngine_io_test.ply";
    error.clear();
    assert(PointCloudIO::savePly(*obj, output.string(), &error));
    assert(error.empty());

    error.clear();
    auto reload = PointCloudIO::loadPly(output.string(), &error);
    assert(reload && error.empty());
    assert(reload->size() == 3); // savePly 只输出未删除点。

    std::error_code ec;
    std::filesystem::remove(output, ec);
    std::filesystem::remove(ascOutput, ec);
}

void testCompactRemap() {
    auto cloud = makeCloud();
    Engine editor(cloud);

    editor.select({1});
    assert(editor.deleteSelection());
    editor.select({2, 3});

    const auto mapping = editor.compact();
    assert(mapping.size() == 4);
    assert(mapping[0] == 0);
    assert(mapping[1] == kInvalidPointId);
    assert(mapping[2] == 1);
    assert(mapping[3] == 2);
    assert(cloud->size() == 3);

    // compact 后 Selection 自动跟随新 PointId。
    assert((editor.selection().ids() == std::vector<PointId>{1, 2}));

    // compact 会使旧历史中的 PointId 失效，所以历史必须清空。
    assert(!editor.canUndo());
    assert(!editor.canRedo());
}

} // namespace

static void testColorPicking24() {
    using namespace JMEngine;
    const std::uint32_t ids[] = {0u, 1u, 254u, 255u, 65534u, 65535u, 10000000u, kColorPicking24MaxObjectId};
    for (const auto id : ids) {
        const auto c = encodeColorId24(id);
        std::uint32_t out = 0;
        const bool ok = decodeColorId24(c.r, c.g, c.b, out);
        assert(ok);
        assert(out == id);
    }
    std::uint32_t background = 123u;
    assert(!decodeColorId24(0, 0, 0, background));
}

static void testBlockPicking24() {
    using namespace JMEngine;

    // 250 万对象会被拆成 100 万 + 100 万 + 50 万三个 block。
    constexpr std::uint64_t total = 2'500'000u;
    static_assert(pickBlockCount(total) == 3u);

    const auto b0 = pickBlockAt(total, 0u);
    const auto b1 = pickBlockAt(total, 1u);
    const auto b2 = pickBlockAt(total, 2u);
    assert(b0.firstObject == 0u && b0.objectCount == 1'000'000u);
    assert(b1.firstObject == 1'000'000u && b1.objectCount == 1'000'000u);
    assert(b2.firstObject == 2'000'000u && b2.objectCount == 500'000u);

    // 模拟第二个 block 内 localId=123456 的 RGB24 编解码。
    constexpr std::uint32_t localId = 123'456u;
    const auto color = encodeBlockLocalId24(localId);
    std::uint64_t globalId = 0u;
    assert(decodeBlockGlobalId24(b1, color.r, color.g, color.b, globalId));
    assert(globalId == 1'123'456u);

    // RGB=(0,0,0) 必须继续表示背景。
    assert(!decodeBlockGlobalId24(b1, 0u, 0u, 0u, globalId));
}

static void testMeshEditSession() {
    auto cloud = makeCloud();
    auto mesh = std::make_shared<TriangleMesh>(cloud, std::vector<std::uint32_t>{0, 1, 2, 1, 2, 3});
    MeshEditSession editor(mesh);

    editor.select({0});
    auto r = editor.deleteSelection();
    assert(r.changed && r.topologyChanged);
    assert(mesh->activeTriangleCount() == 1);
    const auto visibleBuffer = mesh->buildVisibleBuffer();
    assert((visibleBuffer.indices == std::vector<std::uint32_t>{1, 2, 3}));
    // packed primitive 0 当前对应原始 TriangleId 1；GPU Picking 必须通过这个映射回原 ID。
    assert((visibleBuffer.triangleIds == std::vector<TriangleId>{1}));

    r = editor.undo();
    assert(r.changed);
    assert(mesh->activeTriangleCount() == 2);

    r = editor.redo();
    assert(r.changed);
    assert(mesh->activeTriangleCount() == 1);

    editor.undo();
    editor.select({1});
    Mat4f t = Mat4f::identity();
    t.m[12] = 2.0f;
    r = editor.transformSelection(t);
    assert(r.changed && r.geometryChanged);
    assert(cloud->points()[1].position.x == 2.0f);
    assert(cloud->points()[2].position.x == 2.5f);
    assert(cloud->points()[3].position.x == 4.0f);
    editor.undo();
    assert(cloud->points()[1].position.x == 0.0f);

    editor.select({0});
    editor.deleteSelection();
    const auto map = editor.compactTriangles();
    assert(map.size() == 2);
    assert(map[0] == kInvalidTriangleId);
    assert(map[1] == 0);
    assert(mesh->triangleCount() == 1);
}

static void testCpuMeshSurfaceSelector() {
    // 两个屏幕投影完全重叠的三角形：Triangle 0 在前(z=-0.5)，Triangle 1 在后(z=+0.5)。
    // Surface 必须只返回前面；Through 必须返回两个。
    PointCloud::Container pts = {
        {{-0.6f, -0.6f, -0.5f}, 0xffffffffu, PointValid}, {{0.6f, -0.6f, -0.5f}, 0xffffffffu, PointValid},
        {{0.0f, 0.6f, -0.5f}, 0xffffffffu, PointValid},   {{-0.6f, -0.6f, 0.5f}, 0xffffffffu, PointValid},
        {{0.6f, -0.6f, 0.5f}, 0xffffffffu, PointValid},   {{0.0f, 0.6f, 0.5f}, 0xffffffffu, PointValid}};
    auto cloud = std::make_shared<PointCloud>(std::move(pts));
    auto mesh = std::make_shared<TriangleMesh>(cloud, std::vector<std::uint32_t>{0, 1, 2, 3, 4, 5});
    const Mat4f mvp = Mat4f::identity();
    const Viewport vp{200, 200};
    const RectI rect{40, 40, 160, 160};

    const auto through = CpuMeshSelector::rectangle(*mesh, mvp, vp, rect);
    assert((through == std::vector<TriangleId>{0, 1}));

    auto surface = CpuMeshSelector::surfaceRectangle(*mesh, mvp, vp, rect);
    assert((surface == std::vector<TriangleId>{0}));

    // 删除前面三角形后，Surface 必须能看到后面的 Triangle 1，证明使用的是 TriangleFlags。
    MeshEditSession editor(mesh);
    editor.select({0});
    const auto del = editor.deleteSelection();
    assert(del.changed);
    surface = CpuMeshSelector::surfaceRectangle(*mesh, mvp, vp, rect);
    assert((surface == std::vector<TriangleId>{1}));

    // Circle/Lasso/Brush 也必须保持 TriangleId + Z-buffer 语义。
    editor.undo();
    const auto circle = CpuMeshSelector::surfaceCircle(*mesh, mvp, vp, {100, 100}, 50);
    assert((circle == std::vector<TriangleId>{0}));
    const std::vector<Point2i> lasso{{50, 50}, {150, 50}, {150, 150}, {50, 150}};
    const auto ls = CpuMeshSelector::surfaceLasso(*mesh, mvp, vp, lasso);
    assert((ls == std::vector<TriangleId>{0}));
    const std::vector<Point2i> brush{{70, 100}, {130, 100}};
    const auto bs = CpuMeshSelector::surfaceBrushStroke(*mesh, mvp, vp, brush, 20);
    assert((bs == std::vector<TriangleId>{0}));
}

static void testMeshSelectionClosure() {
    // 一个由 4 个共面三角形组成的连续条带。模拟远距离 GPU 只命中首尾两个三角形，
    // 中间两个亚像素三角形没有写入 Picking FBO；闭合后必须补齐整个同表面局部。
    PointCloud::Container pts = {
        {{0.f, 0.f, 0.f}, 0xffffffffu, PointValid}, {{1.f, 0.f, 0.f}, 0xffffffffu, PointValid},
        {{0.f, 1.f, 0.f}, 0xffffffffu, PointValid}, {{1.f, 1.f, 0.f}, 0xffffffffu, PointValid},
        {{2.f, 0.f, 0.f}, 0xffffffffu, PointValid}, {{2.f, 1.f, 0.f}, 0xffffffffu, PointValid}};
    auto cloud = std::make_shared<PointCloud>(std::move(pts));
    // 两个方格，每格两个三角形，共 4 面，全部共享边连续。
    auto mesh = std::make_shared<TriangleMesh>(cloud, std::vector<std::uint32_t>{0, 1, 2, 1, 3, 2, 1, 4, 3, 4, 5, 3});
    const std::vector<TriangleId> seeds{0, 3};
    const std::vector<TriangleId> candidates{0, 1, 2, 3};
    MeshSelectionClosureOptions opt;
    opt.maxRings = 8;
    opt.minAdjacentNormalDot = 0.5f;
    const auto closed = MeshSelectionClosure::expandSurfaceSelection(*mesh, seeds, candidates, opt);
    assert((closed == std::vector<TriangleId>{0, 1, 2, 3}));

    // 候选区域外的面绝不能被闭合带入。
    const std::vector<TriangleId> limitedCandidates{0, 1};
    const auto limited = MeshSelectionClosure::expandSurfaceSelection(*mesh, {0}, limitedCandidates, opt);
    assert((limited == std::vector<TriangleId>{0, 1}));
}

static void testProcessingOperations() {
    using namespace JMEngine;
    using namespace JMEngine::processing;

    // Voxel：两个极近点应合并。
    PointCloud::Container pts = {{{0.0000f, 0, 0}, 0xff0000ffu, PointValid},
                                 {{0.0002f, 0, 0}, 0xff00ff00u, PointValid},
                                 {{0.0100f, 0, 0}, 0xffff0000u, PointValid},
                                 {{0.0200f, 0, 0}, 0xffffffffu, PointValid}};
    auto cloud = std::make_shared<PointCloud>(pts);
    auto voxel = createOperation("voxel");
    assert(voxel);
    ParameterMap vp;
    vp["voxel_mm"] = 1.0;
    auto vr = voxel->run({cloud, {}}, vp, {}, {});
    assert(vr.success && vr.cloud);
    assert(vr.outputPoints == 3);

    // Radius：远离主体的单独噪点应被删掉。
    PointCloud::Container radiusPts = {{{0.000f, 0, 0}, 0xffffffffu, PointValid},
                                       {{0.001f, 0, 0}, 0xffffffffu, PointValid},
                                       {{0.002f, 0, 0}, 0xffffffffu, PointValid},
                                       {{0.100f, 0, 0}, 0xffffffffu, PointValid}};
    auto radiusCloud = std::make_shared<PointCloud>(radiusPts);
    auto radius = createOperation("radius_outlier");
    ParameterMap rp;
    rp["radius_mm"] = 3.0;
    rp["min_neighbors"] = std::int64_t(1);
    auto rr = radius->run({radiusCloud, {}}, rp, {}, {});
    assert(rr.success && rr.outputPoints == 3);

    // Small cluster：3 点主体 + 1 点远离小簇，只保留主体。
    auto cluster = createOperation("small_cluster");
    ParameterMap cp;
    cp["radius_mm"] = 3.0;
    cp["min_points"] = std::int64_t(2);
    auto cr = cluster->run({radiusCloud, {}}, cp, {}, {});
    assert(cr.success && cr.outputPoints == 3);

    // Normal estimation 应产生至少一个非零法向。
    PointCloud::Container planePts = {{{0, 0, 0}, 0xffffffffu, PointValid},
                                      {{0.001f, 0, 0}, 0xffffffffu, PointValid},
                                      {{0, 0.001f, 0}, 0xffffffffu, PointValid},
                                      {{0.001f, 0.001f, 0}, 0xffffffffu, PointValid}};
    auto plane = std::make_shared<PointCloud>(planePts);
    auto normalOp = createOperation("normal_estimation");
    ParameterMap np;
    np["knn"] = static_cast<std::int64_t>(3);
    auto nr = normalOp->run({plane, {}}, np, {}, {});
    assert(nr.success && nr.cloud);
    bool hasNormal = false;
    for (const auto& p : nr.cloud->points()) {
        const float n2 = p.normal.x * p.normal.x + p.normal.y * p.normal.y + p.normal.z * p.normal.z;
        if (n2 > 0.5f)
            hasNormal = true;
    }
    assert(hasNormal);

    // Mesh cleanup：退化面 + 重复面应被清理，只留一个正常面。
    auto meshCloud = std::make_shared<PointCloud>(PointCloud::Container{{{0, 0, 0}, 0xffffffffu, PointValid},
                                                                        {{1, 0, 0}, 0xffffffffu, PointValid},
                                                                        {{0, 1, 0}, 0xffffffffu, PointValid},
                                                                        {{1, 1, 0}, 0xffffffffu, PointValid}});
    auto dirtyMesh = std::make_shared<TriangleMesh>(meshCloud, std::vector<std::uint32_t>{0, 1, 2, 0, 1, 2, 0, 0, 3});
    auto cleanup = createOperation("mesh_cleanup");
    auto clr = cleanup->run({meshCloud, dirtyMesh}, {}, {}, {});
    assert(clr.success && clr.mesh && clr.outputTriangles == 1);

    // 量产诊断：原 dirtyMesh 包含一个退化三角形，且单个有效三角形具有 3 条边界边。
    const auto diagnostics = analyzeModel({meshCloud, dirtyMesh});
    assert(diagnostics.points == 4);
    assert(diagnostics.triangles == 3);
    assert(diagnostics.degenerateTriangles >= 1);

    auto boundaryMesh = std::make_shared<TriangleMesh>(meshCloud, std::vector<std::uint32_t>{0, 1, 2});
    const auto boundaryDiagnostics = analyzeModel({meshCloud, boundaryMesh});
    assert(boundaryDiagnostics.boundaryEdges == 3);
    assert(boundaryDiagnostics.nonManifoldEdges == 0);
    assert(boundaryDiagnostics.connectedComponents == 1);

    // Smooth 不应改变三角形数量。
    auto smoothMesh = std::make_shared<TriangleMesh>(meshCloud, std::vector<std::uint32_t>{0, 1, 2, 1, 3, 2});
    ParameterMap sp;
    sp["iterations"] = std::int64_t(2);
    auto taubin = createOperation("taubin");
    auto sr = taubin->run({meshCloud, smoothMesh}, sp, {}, {});
    assert(sr.success && sr.outputTriangles == 2);

    // 简化：4 个三角形保留 50%，输出应为 2。
    auto decCloud = std::make_shared<PointCloud>(PointCloud::Container{{{0, 0, 0}, 0xffffffffu, PointValid},
                                                                       {{1, 0, 0}, 0xffffffffu, PointValid},
                                                                       {{0, 1, 0}, 0xffffffffu, PointValid},
                                                                       {{1, 1, 0}, 0xffffffffu, PointValid},
                                                                       {{2, 0, 0}, 0xffffffffu, PointValid},
                                                                       {{2, 1, 0}, 0xffffffffu, PointValid}});
    auto decMesh =
        std::make_shared<TriangleMesh>(decCloud, std::vector<std::uint32_t>{0, 1, 2, 1, 3, 2, 1, 4, 3, 4, 5, 3});
    ParameterMap dp;
    dp["ratio"] = 0.5;
    dp["preserve_boundary"] = false;
    auto dec = createOperation("qem_decimate");
    auto dr = dec->run({decCloud, decMesh}, dp, {}, {});
    assert(dr.success && dr.outputTriangles <= 2 && dr.outputTriangles > 0);

    // Hole fill：一个单独三角形的边界环会被检测，并在 fill=true 时增加三角形。
    auto holeCloud = std::make_shared<PointCloud>(PointCloud::Container{{{0, 0, 0}, 0xffffffffu, PointValid},
                                                                        {{1, 0, 0}, 0xffffffffu, PointValid},
                                                                        {{0, 1, 0}, 0xffffffffu, PointValid}});
    auto holeMesh = std::make_shared<TriangleMesh>(holeCloud, std::vector<std::uint32_t>{0, 1, 2});
    auto hole = createOperation("hole_fill");
    ParameterMap hp;
    hp["max_edges"] = std::int64_t(10);
    hp["fill"] = true;
    auto hr = hole->run({holeCloud, holeMesh}, hp, {}, {});
    assert(hr.success && hr.holesDetected >= 1);
    assert(hr.outputTriangles > hr.inputTriangles);

    // 自适应默认参数：模型尺度放大后，Voxel 默认体素也必须同步增大。
    auto adaptiveVoxel = createOperation("voxel");
    PointCloud::Container adaptivePts = {{{0, 0, 0}, 0xffffffffu, PointValid},
                                         {{0.001f, 0, 0}, 0xffffffffu, PointValid},
                                         {{0, 0.001f, 0}, 0xffffffffu, PointValid},
                                         {{0, 0, 0.001f}, 0xffffffffu, PointValid},
                                         {{0.001f, 0.001f, 0.001f}, 0xffffffffu, PointValid}};
    auto adaptiveCloud = std::make_shared<PointCloud>(adaptivePts);
    const auto smallDesc = estimateOperationDescriptor(*adaptiveVoxel, {adaptiveCloud, {}});
    auto largeCloud = std::make_shared<PointCloud>(adaptivePts);
    for (auto& point : largeCloud->points()) {
        point.position.x *= 100.0f;
        point.position.y *= 100.0f;
        point.position.z *= 100.0f;
    }
    const auto largeDesc = estimateOperationDescriptor(*adaptiveVoxel, {largeCloud, {}});
    auto defaultValue = [](const OperationDescriptor& desc, const char* key) {
        for (const auto& spec : desc.parameters)
            if (spec.key == key)
                return spec.defaultValue;
        return 0.0;
    };
    assert(defaultValue(largeDesc, "voxel_mm") > defaultValue(smallDesc, "voxel_mm") * 20.0);

    // 八叉树泊松：构造带外向法向的球面点集，必须输出非空 TriangleMesh。
    PointCloud::Container spherePts;
    constexpr int rings = 8;
    constexpr int sectors = 16;
    for (int r0 = 1; r0 < rings; ++r0) {
        const float theta = 3.14159265358979323846f * float(r0) / float(rings);
        for (int s0 = 0; s0 < sectors; ++s0) {
            const float phi = 2.0f * 3.14159265358979323846f * float(s0) / float(sectors);
            Vec3f n{std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
            Point q;
            q.position = n;
            q.normal = n;
            spherePts.push_back(q);
        }
    }
    auto sphere = std::make_shared<PointCloud>(std::move(spherePts));
    auto poisson = createOperation("poisson_octree");
    assert(poisson);
    ParameterMap pp;
    pp["resolution_mm"] = 20.0;
    pp["max_depth"] = std::int64_t(9);
    pp["full_depth"] = std::int64_t(4);
    pp["samples_per_node"] = 0.5;
    pp["point_weight"] = 4.0;
    pp["iterations"] = std::int64_t(8);
    pp["estimate_normals"] = false;
    pp["orient_normals"] = true;

    // Poisson 预检按 max_depth 给出保守工作集估算；真实自动 Depth 在 worker 内由 resolution+BBox 计算。
    const auto pf = preflightOperation(*poisson, {sphere, {}}, pp);
    assert(pf.diagnostics.points == sphere->activeCount());
    assert(pf.requestedDepth == 9);
    assert(pf.estimatedWorkingSetBytes > 0);
    assert(pf.allowed);

    auto pr = poisson->run({sphere, {}}, pp, {}, {});
    if (industrialPoissonAvailable())
        assert(pr.success && pr.mesh && pr.outputTriangles > 0);
    else
        assert(!pr.success && !pr.message.empty());

    // OpenMP 策略必须至少留 1 个逻辑核；单核机器退化为 1。
    assert(processingThreadCount() >= 1);
}

class TestScanBackend final : public JMEngine::ISlam {
  public:
    bool process(const JMEngine::CameraFrame& frame) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        JMEngine::Point p; p.position={float(frame.frameId),0,0};
        cloud_=std::make_shared<JMEngine::PointCloud>(JMEngine::PointCloud::Container{p});
        //if(update_)update_(frame.frameId,{},cloud_);
        return true;
    }
    JMEngine::Pose pose()const override{return{};}
    std::shared_ptr<JMEngine::PointCloud>cloud()override{return cloud_;}
    void setUpdateCallback(UpdateCallback cb)override{update_=std::move(cb);}
  private:
    UpdateCallback update_;std::shared_ptr<JMEngine::PointCloud>cloud_{new JMEngine::PointCloud};
};

void testScannerStateAndBackpressure(){
    JMEngine::JMScanner scanner(std::unique_ptr<JMEngine::ISlam>(new TestScanBackend));
    JMEngine::ScanConfig cfg;cfg.maxFrames=100;cfg.maxInflightFrames=1;
    assert(scanner.initialize(cfg));assert(scanner.state()==JMEngine::ScanState::Idle);assert(scanner.start());
    auto rgb=std::make_shared<std::vector<std::uint8_t>>(12,127);auto code=std::make_shared<std::vector<std::uint8_t>>(4,64);
    for(int i=0;i<30;++i){JMEngine::CameraFrame f;f.rgb=rgb;f.code=code;f.width=2;f.height=2;f.frameId=i;assert(scanner.submit(std::move(f)));}
    std::this_thread::sleep_for(std::chrono::milliseconds(30));scanner.stop();auto stats=scanner.statistics();
    assert(scanner.state()==JMEngine::ScanState::ReadyForReconstruction);assert(stats.submittedFrames==30);assert(stats.replacedFrames>0);assert(stats.processedFrames>0);
    assert(scanner.reconstruct());assert(scanner.resultCloud());scanner.reset();assert(scanner.state()==JMEngine::ScanState::Idle);
}

int main() {
    testScannerStateAndBackpressure();
    testColorPicking24();
    testBlockPicking24();
    testMeshEditSession();
    testCpuMeshSurfaceSelector();
    testMeshSelectionClosure();
    testProcessingOperations();
    testSelectionDeleteUndoRedo();
    testCrop();
    testTransform();
    testReplaceCloudClearsEditState();
    testCpuSelector();
    testPixelIdPicker();
    testPointCloudIo();
    testCompactRemap();

    std::cout << "All JMEngine core tests passed.\n";
    {
        JMEngine::Point p0;
        p0.position = {0, 0, 0};
        JMEngine::Point p1;
        p1.position = {1, 0, 0};
        JMEngine::Point p2;
        p2.position = {0, 1, 0};
        auto cloud = std::make_shared<JMEngine::PointCloud>(JMEngine::PointCloud::Container{p0, p1, p2});
        JMEngine::TriangleMesh mesh(cloud, {0, 1, 2});
        assert(JMEngine::recomputeVertexNormals(mesh));
        for (const auto& p : cloud->points())
            assert(p.normal.z > 0.99f);
    }

    {
        const auto temp = std::filesystem::temp_directory_path() / "JMEngine_txt_import_221.txt";
        {
            std::ofstream out(temp);
            out << "0 0 0 255 0 0 0 0 1\n";
            out << "1,0,0,0,255,0,0,0,1\n";
        }
        std::string error;
        auto cloud = JMEngine::ModelIO::loadTxt(temp.string(), &error);
        assert(cloud && cloud->size() == 2);
        assert(cloud->points()[0].normal.z > 0.99f);
        std::filesystem::remove(temp);
    }
    {
        JMEngine::Point p0;
        p0.position = {0, 0, 0};
        JMEngine::Point p1;
        p1.position = {1, 0, 0};
        JMEngine::Point p2;
        p2.position = {0, 1, 0};
        auto cloud = std::make_shared<JMEngine::PointCloud>(JMEngine::PointCloud::Container{p0, p1, p2});
        JMEngine::TriangleMesh mesh(cloud, {0, 1, 2});
        assert(JMEngine::recomputeVertexNormals(mesh));
        const auto base = std::filesystem::temp_directory_path() / "JMEngine_export_221";
        std::string error;
        assert(JMEngine::ModelIO::save(*cloud, &mesh, (base.string() + ".obj"), &error));
        assert(JMEngine::ModelIO::save(*cloud, &mesh, (base.string() + ".stl"), &error));
        assert(JMEngine::ModelIO::save(*cloud, &mesh, (base.string() + ".ply"), &error));
        assert(std::filesystem::file_size(base.string() + ".obj") > 0);
        assert(std::filesystem::file_size(base.string() + ".stl") > 0);
        assert(std::filesystem::file_size(base.string() + ".ply") > 0);
        std::filesystem::remove(base.string() + ".obj");
        std::filesystem::remove(base.string() + ".stl");
        std::filesystem::remove(base.string() + ".ply");
    }

    {
        // 2.2.2 回归：网格模型切到“点显示”后按 PointId 删除，
        // 任何引用该点的三角形都必须从 Visible EBO 中消失。
        JMEngine::Point p0;
        p0.position = {0, 0, 0};
        JMEngine::Point p1;
        p1.position = {1, 0, 0};
        JMEngine::Point p2;
        p2.position = {0, 1, 0};
        JMEngine::Point p3;
        p3.position = {1, 1, 0};
        auto cloud = std::make_shared<JMEngine::PointCloud>(JMEngine::PointCloud::Container{p0, p1, p2, p3});
        JMEngine::TriangleMesh mesh(cloud, {0, 1, 2, 1, 3, 2});
        assert(mesh.activeTriangleCount() == 2);
        cloud->points()[0].flags |= JMEngine::PointDeleted;
        const auto visible = mesh.buildVisibleBuffer();
        assert(mesh.activeTriangleCount() == 1);
        assert(visible.triangleIds.size() == 1);
        assert(visible.triangleIds[0] == 1);
        assert(visible.indices.size() == 3);
    }

    {
        // 2.2.3 回归：CPU Surface 必须与 Through 严格区分。两个点投影到同一像素时，
        // Through 选择两点；Surface 只保留最前深度附近的点。
        JMEngine::Point front;
        front.position = {0.0f, 0.0f, -0.5f};
        JMEngine::Point back;
        back.position = {0.0f, 0.0f, 0.5f};
        JMEngine::PointCloud cloud({front, back});
        const auto mvp = JMEngine::Mat4f::identity();
        const JMEngine::Viewport vp{100, 100};
        const JMEngine::RectI rect{40, 40, 60, 60};
        const auto through = JMEngine::CpuSelector::rectangle(cloud, mvp, vp, rect);
        const auto surface = JMEngine::CpuSelector::rectangleSurface(cloud, mvp, vp, rect, 0.001f);
        assert(through.size() == 2);
        assert(surface.size() == 1);
        assert(surface[0] == 0);
    }

    {
        // 2.2.6 回归：前点的 4px point-sprite 覆盖后点中心时，即使两个点中心落在相邻像素，
        // CPU Surface 也必须剔除后点；旧版只写中心 1 像素会错误选中两点。
        JMEngine::Point front;
        front.position = {0.0f, 0.0f, -0.5f};
        JMEngine::Point back;
        back.position = {0.02f, 0.0f, 0.5f};
        JMEngine::PointCloud cloud({front, back});
        const auto mvp = JMEngine::Mat4f::identity();
        const JMEngine::Viewport vp{100, 100};
        const JMEngine::RectI rect{40, 40, 60, 60};
        const auto through = JMEngine::CpuSelector::rectangle(cloud, mvp, vp, rect);
        const auto surface = JMEngine::CpuSelector::rectangleSurface(cloud, mvp, vp, rect, 0.0005f);
        assert(through.size() == 2);
        assert(surface.size() == 1);
        assert(surface[0] == 0);
    }

    {
        auto poisson = JMEngine::processing::createOperation("poisson_octree");
        assert(poisson);
        assert(poisson->descriptor().outputPolicy == JMEngine::processing::OutputPolicy::AddModelOnKindChange);
        const auto poissonDesc = poisson->descriptor();
        const auto colorIt = std::find_if(poissonDesc.parameters.begin(), poissonDesc.parameters.end(),
                                          [](const auto& spec) { return spec.key == "use_input_color"; });
        assert(colorIt != poissonDesc.parameters.end());
        assert(colorIt->kind == JMEngine::processing::ParameterKind::Boolean);
        assert(colorIt->defaultValue == 1.0);
    }

    return 0;
}
