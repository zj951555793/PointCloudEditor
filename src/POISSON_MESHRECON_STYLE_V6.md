# Poisson MeshRecon-style V6

本版本以 KNN normals V5 为基线，将工业 Poisson 参数语义改成与 meshRecon 类似的“物理分辨率驱动”。

## 重建流程

1. 输入点云与 KNN PCA 法线检查。
2. 根据输入 BBox 最大边长和 `resolution_mm` 自动计算 Depth。
3. Scale 不再固定 1.05/1.10；只保留约一个目标分辨率单元的边界 padding。
4. 官方 PoissonRecon 18.76 Screened Poisson 求解。
5. 输出 Poisson vertex density。
6. 使用绝对 `trim_value` 做 density trim（不再使用最低百分位）。
7. 按实际网格总面积比例 `island_area_ratio` 删除小孤岛，不再按固定三角形数量判断。
8. 仅执行基础网格清理与最终法线重建。

不会恢复历史版本的 nearest-point-distance 后裁剪。

## 推荐参数

大型家具/沙发扫描初始建议：

- 目标重建精度：5.0 mm
- 最大八叉树深度：11
- FullDepth：5
- SamplesPerNode：0.5
- PointWeight：4.0
- Density Trim：2.0
- IslandAreaRatio：0.00005
- LinearFit：开启
- 兼容旧版尺寸校正：关闭

如果原始扫描足够密并希望保留雕花细节，可将精度逐步改成 3 mm / 2 mm；不要直接提高最大 Depth。

`SamplesPerNode=0.2` 更接近 meshRecon 示例中的细节模式，但会更容易保留噪声，建议仅在干净点云上使用。

## 自动 Depth / Scale

内部核心逻辑：

```
resolutionModel = resolution_mm -> 当前模型坐标单位
scale = clamp(1 + 2 * resolutionModel / maxExtent, 1.005, 1.08)
depth = ceil(log2(maxExtent * scale / resolutionModel))
depth = clamp(depth, 6, max_depth)
```

这样 Scale 的作用仅是给 Poisson BBox 留最小必要 padding，而不是无条件把求解域扩大 5%~10%。

完成日志会打印实际使用的 `resolution / autoDepth / autoScale / trim / islandRatio`，便于现场比较不同数据集。
