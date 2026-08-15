# Poisson V8 - Official SurfaceTrimmer

## 目标

V8 不再维护 PointCloudEditor 自己实现的 SurfaceTrimmer。工业泊松流程直接使用工程已经配置的官方 PoissonRecon 源码，并由 CMake 原样编译 `Src/SurfaceTrimmer.cpp`。

流程：

1. 输入点云（已有法线直接使用；缺法线时 KNN/PCA 补齐）。
2. 官方 PoissonRecon 求解，并在 level-set extraction 时输出 density。
3. 把 `x y z value(density)` 和 polygon 写入临时 ASCII PLY。
4. 直接调用 CMake 生成的官方 `pceditor_surface_trimmer_official`。
5. 读取官方 SurfaceTrimmer 输出 Mesh。
6. 重建最终 Mesh 法线。
7. 若勾选“使用输入颜色”，将输入点云 RGB 最近邻传递到最终 Mesh，并关闭 Qt 的中性灰显示覆盖。

## CMake

继续使用项目已有的 PoissonRecon 路径配置：

- `third_party/PoissonRecon`，或
- `PCEDITOR_POISSONRECON_ROOT=<官方 PoissonRecon 根目录>`

只要该目录存在：

- `Src/Reconstructors.h`
- `Src/SurfaceTrimmer.cpp`

CMake 会额外生成：

`pceditor_surface_trimmer_official`

算法实现没有复制/改写，直接编译官方 `SurfaceTrimmer.cpp`。

Qt editor 构建后会把该可执行文件复制到 `pceditor_qt_editor` 同目录；运行时优先使用 CMake 生成路径，部署后回退查找应用程序同目录。

## SurfaceTrimmer 参数

- `SurfaceTrimmer Trim`: 直接传给官方 `--trim`
- `SurfaceTrimmer 岛面积比`: 直接传给官方 `--aRatio`
- `SurfaceTrimmer 删除小岛`: 对应官方 `--removeIslands`

`Trim=0` 时绕过 SurfaceTrimmer，直接使用原始 Poisson Mesh。

## 输入颜色修复

旧版 Poisson 已经执行 RGB 传递，但 `PointCloudWidget::Model(mesh)` 默认设置：

`useDisplayColor = true`

因此新 Mesh 被统一灰色覆盖，用户看不到真实 RGB。

V8 在 `use_input_color=true` 时显式：

`newModel->useDisplayColor = false`

因此勾选“使用输入颜色”后会直接显示 `Point::rgba`；未勾选时仍沿用中性灰材质。

## 法线

V8 继续保持 V7 的要求：Poisson 路径不再执行“法向一致化/质心朝外”二次翻转。已有法线原样使用；只在法线缺失时执行 KNN/PCA 法线估计。
