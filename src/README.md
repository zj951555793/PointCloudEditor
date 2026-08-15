# PointCloudEditor 2.2.8

## 2.2.8 - GPU 点框选回滚修复

- 点模式 GPU Surface 框选回滚到 2.2.2 的稳定 R32UI FBO + Depth + glReadPixels 实现。
- 移除 2.2.3~2.2.6 引入的 Compute/SSBO 点框选代码与运行资源。
- GPU 不可用时仍走 CPU fallback；Through 继续使用 CPU 全投影。
- 网格 TriangleId R32UI Picking、Poisson、颜色、光照、I/O、模型管理器保持不变。


## 2.2.8 Selection 修复

- GPU 点框选结果改为 `selectedFlags[PointId]` SSBO，一点一标志位，移除 atomic append/counter 回读路径。
- CPU Surface 使用与 GPU Picking 一致的 4px point-sprite footprint 建软件深度图，不再因只记录点中心像素而接近 Through。
- Surface 深度容差收紧为 0.0005。
- Surface/Through 降级保持原语义：GPU Surface 失败只降级 CPU Surface；Through 只降级 CPU Through。
- 状态提示只显示当前真实模式：`CPU / Surface`、`CPU / Through`、`GPU Compute / Surface`、`GPU Compute / Through`。



## 2.2.8 Selection + Appearance 稳定版

本修复版补充：

- 修复 Qt6 `QListWidget::itemDoubleClicked` 信号槽参数不匹配导致的 `SlotReturnType / AreArgumentsCompatible / connectImpl` 编译错误。
- 工业泊松重建新增 `使用输入颜色` 选项，默认开启。开启时在最终 Mesh Cleanup 与法线重建之后，通过空间哈希最近邻把输入点云 RGB 传递到 Poisson Mesh 顶点；关闭时不传递输入 RGB，使用模型显示色。
- 颜色传递继续遵守 `CPU-1` 并行策略。


- CPU Surface 使用软件深度图，和 Through 选择严格区分。
- 点模式 GPU 选择使用 Compute Shader + SSBO 全点逐个投影；远距离多个点落到同一像素也不会因 ID FBO 覆盖而漏选。Desktop GL 4.3+ / GLES 3.1 可用 Compute；能力不足自动 CPU。
- 网格模型默认 Solid，点云默认 Points。
- 模型管理器支持每模型独立显示颜色，颜色只影响 GPU 显示，不修改原始 RGB。
- PointCloud -> Mesh（工业泊松）新增 Mesh 模型并保留源点云；同类型 Processing 继续替换当前模型。
- Mesh 使用 MeshLab 风格中性材质与 Camera Headlight；主光随 Orbit 相机每帧更新，降低过曝并改善背面可读性。

## 2.2.8 显示 / I/O / Poisson 法线修复

- Poisson 在 Density Trim 与 Mesh Cleanup 后强制面积加权重建最终顶点法线，避免输出网格 `normal=0`。
- Desktop Picking 移除旧 `GLSL 120 + EXT_gpu_shader4` 整数路径：OpenGL 3.2+ 使用 GLSL 150 + R32UI；否则自动 CPU。
- 新增点、实体网格、线框、实体+线框显示模式；线框使用 `GL_LINES` EBO，兼容 Desktop/GLES，不依赖 `glPolygonMode`。
- 新增统一导出：PLY / ASC / OBJ / STL；ASC 采用流式 XYZRGB 文本输出，网格导出会写真实三角面。
- 新增 TXT / ASC 点云导入：自动识别空格/Tab/逗号/分号分隔的 XYZ、XYZI、XYZRGB、XYZRGBNormal；XYZI 的 intensity 当前明确忽略，避免误当颜色。

# PointCloudEditor 2.2.8

## 2.2.8 - Production Stability / Diagnostics

本版本不继续堆叠处理算法，重点把 2.1.x 的 Processing / Industrial Poisson 收口为量产稳定架构。

### 模型诊断

菜单：`工具 -> 模型诊断...`。诊断任务复用现有 `PointCloudWidget::workerPool_`，不会在 UI 线程扫描百万三角形。

Core 新增 `pceditor::processing::Diagnostics`：

- 有效点、软删除点、NaN/Inf 坐标；
- 有效法向数量 / 法向覆盖率；
- BBox 对角线与估计点间距；
- 网格有效/删除三角形；
- 退化三角形；
- boundary edges；
- non-manifold edges；
- connected components。

### Processing Preflight / Poisson 内存保护

点击“开始”前会执行统一预检。Poisson 根据有效点数、Depth、SamplesPerNode 对 adaptive octree / FEM solver /
临时向量 / 输出网格工作集做**保守内存估算**，并与当前系统可用物理内存比较。

- 预计工作集超过安全预算时，不直接启动；
- 自动给出推荐的更低 Depth；
- 高内存占用但仍在预算内时给 warning，可由用户确认继续；
- NaN/Inf、低法向覆盖率、点数过少也会进入预检警告。

该估算用于防止 OOM，不声称等同于 PoissonRecon 实际峰值 profiler 数据。

### 每模型 Processing Undo/Redo

2.1.x 的 Processing 历史是全局栈，多模型交替处理时可能互相阻塞。2.2.8 改为每个 `Model` 独立：

```text
Model A -> processingUndo / processingRedo
Model B -> processingUndo / processingRedo
```

每个模型最多保留 4 个大处理快照，快照继续使用 `shared_ptr`，避免深拷贝；历史有上限，避免多次 Poisson/QEM
后旧几何长期占用无限内存。普通点/面编辑的细粒度 Undo/Redo 优先级保持不变。

### 重处理任务互斥 / Poisson 阶段化进度

模型诊断和 Processing 共用现有 worker pool，并通过 busy 状态互斥，避免同一编辑器同时启动多个大内存任务。
Poisson 进度显示为：

```text
1/6 PCA 法向估计
2/6 初始化 PoissonRecon
3/6 自适应八叉树 FEM 求解
4/6 提取等值面与密度
5/6 网格清理
6/6 完成
```

Poisson 输出后还会在 worker 线程执行健康度统计，完成摘要会报告退化面、非流形边、边界边和连通域。

### 兼容性

- 保留 MSVC `/bigobj`；
- 保留 C++17；
- 保留 OpenMP `max(1, logicalCPUCount - 1)`；
- 保留 Desktop OpenGL 2.1 / RK3588 GLES3.1 渲染架构；
- 保留 PoissonRecon 18.76 vendor / 离线构建策略；
- Core 不依赖 Qt/OpenGL。


## 2.1.6 - MSVC /bigobj Fix

- Windows/MSVC 全工程统一启用 `/bigobj`，修复 PoissonRecon 重模板实例化可能触发的 `C1128: 节数超过对象文件格式限制`。
- `/bigobj` 通过顶层 `add_compile_options()` 应用于 core、Qt editor、tests 和 examples，避免只修某一个 target 后其它目标再次触发同类问题。
- 升级后建议删除旧 build 目录并重新 CMake Configure/Generate，确保所有 `.obj` 都按 `/bigobj` 重新生成。

## 2.1.6 - Offline Vendor-Ready Industrial Poisson

本版本把 2.1.3 的简化自研 Octree Poisson **彻底移除**，改为集成 Michael Kazhdan 官方
PoissonRecon / Adaptive Multigrid Solvers 18.76 的 `Reconstructors.h` 内存接口。

### 工业泊松重建

菜单：`网格 -> 重建 -> 工业泊松重建...`

处理链：

1. 点云有效性检查；
2. 缺失法向时调用 Core PCA Normal Estimation；
3. 邻域法向符号传播 + 全局一致化；
4. 官方 PoissonRecon 自适应 Octree FEM / screened Poisson 求解；
5. 官方 level-set extraction 输出 TriangleMesh；
6. 输出 vertex density；
7. 低密度 percentile trim；
8. Mesh Cleanup；
9. UI 线程替换模型，Processing Undo/Redo 可完整回退。

Poisson 不写临时 PLY，也不启动外部 `PoissonRecon.exe`。输入/输出通过内存 stream 与 `pceditor` 数据结构连接。

### 参数

统一 ProcessingDialog 根据当前模型自动给出默认值：

- 最大八叉树深度 `Depth`；
- `Full Depth`；
- `Samples Per Node`；
- screened Poisson `Point Weight`；
- `Scale`；
- 每层 Gauss-Seidel `Iterations`；
- `CG Accuracy`；
- Density Trim 百分位；
- 自动法向估计 / 法向一致化；
- 重建后 Mesh Cleanup。

Depth 默认根据有效点数自动选择 8~11，不会默认把大模型直接推到 12/13。

### CPU / UI 策略

官方 PoissonRecon 自带 ThreadPool，但默认线程数等于 `hardware_concurrency()`。为了继续遵守工程统一策略，
CMake 会把官方 header-only `Src` 复制到 build tree，并仅给 `MultiThreading.h` 增加一个
`ThreadPool::SetNumThreads()` setter。求解前传入：

```cpp
max(1, logicalCpuCount - 1)
```

Poisson 的 FEM / Octree / solver / level-set extraction 代码不做修改。

所有处理仍然运行在已有 `PointCloudWidget::workerPool_` 中；Qt/UI/OpenGL 线程只负责参数、进度、模型切换和 GPU 更新。

### PoissonRecon 18.76 依赖

2.2.8 继续采用 **工程内 vendor** 模式，不再要求 cmake-gui 联网。正式目录：

```text
third_party/PoissonRecon/Src/Reconstructors.h
```

一次性导入官方 18.76：

```bash
python tools/vendor_poissonrecon.py
```

Windows 也可以直接：

```bat
tools\vendor_poissonrecon.bat
```

如果你已经手工下载官方 `AdaptiveSolvers.zip`：

```bash
python tools/vendor_poissonrecon.py --archive D:/downloads/AdaptiveSolvers.zip
```

Python/urllib 会继承 `HTTP_PROXY` / `HTTPS_PROXY`。导入完成后删除旧 build 目录，再重新 Configure / Generate。

CMake 默认：

```text
PCEDITOR_FETCH_POISSONRECON = OFF
PCEDITOR_POISSONRECON_VENDOR_DIR = <工程>/third_party/PoissonRecon
```

量产/Jenkins 推荐：

```text
PCEDITOR_REQUIRE_POISSONRECON = ON
```

这样如果 vendor 源码缺失，CMake 会直接失败，而不是生成一个没有工业泊松后端的产品。开发环境仍保留 `PCEDITOR_POISSONRECON_ROOT` 外部源码覆盖和可选 FetchContent fallback。

> 注意：当前发布环境无法取得官方 ZIP 的二进制内容，因此本压缩包包含完整的一键 vendor 工具和离线 CMake 结构，但 **不虚报为已经内置官方源码**。运行上述 vendor 命令一次后，整个工程即可作为完全离线源码树复制。


## 第三方许可

见 `THIRD_PARTY_NOTICES.md` 与 `third_party/PoissonRecon/README.md`。官方仓库顶层 `LICENSE` 为 MIT，
部分历史源码文件还保留 BSD-style 版权/许可头；PointCloudEditor 不移除这些上游声明。

## 本地验证说明

当前构建容器无法访问 GitHub/JHU 二进制源码下载，因此本次在本地实际验证的是：

- 不带官方后端时 Core / Processing / UI 接口仍可编译；
- 不会再编译或调用旧的简化 Octree Poisson；
- CMake 官方后端接入、参数适配和 Adapter 源码已加入发布包。

建议 Windows 第一次构建 2.2.8 时删除旧 build 目录后重新 Configure，使 PoissonRecon 依赖和 CMake patch 从干净状态生成。
## Industrial texture mapping

See `TEXTURE_INDUSTRIAL_FINAL.md` for the final production texture-mapping pipeline, recommended High/Ultra settings, image-quality filtering, seam-aware exposure leveling, and atlas gutter behavior.

## Poisson 保持原始孔洞（不补洞）

工业泊松新增原始扫描点支撑裁剪，默认开启。Poisson 等值面提取后，仅保留有真实输入点云支撑的三角形，可删除大孔洞补片、底部封板和跨未扫描区域的桥接面。参数包括 `保持原始扫描孔洞(不补洞)`、`孔洞支撑距离(0=自动)` 和 `自动支撑距离/点间距`。详细说明见 `POISSON_PRESERVE_HOLES.md`。

## Poisson V7 SurfaceTrimmer

工业泊松重建现在使用连续 density 等值裁剪（SurfaceTrimmer SplitPolygon 语义），并移除了 Poisson 前的法向一致化/质心翻转。详见 `POISSON_SURFACE_TRIMMER_V7.md`。

## Poisson V8 - 官方 SurfaceTrimmer + 输入颜色修复

工业泊松不再使用项目自写的 density 裁剪实现。CMake 直接从已配置的官方 PoissonRecon 源码编译 `Src/SurfaceTrimmer.cpp`，Poisson `--density` 等价输出经临时 PLY 直接交给官方 SurfaceTrimmer。`use_input_color` 同时修复了 Qt 新 Mesh 灰色显示覆盖问题。详见 `POISSON_OFFICIAL_SURFACE_TRIMMER_V8.md`。
