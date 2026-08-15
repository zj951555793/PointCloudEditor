# Poisson V4：取消几何后裁剪

本版本针对复杂扫描中“Poisson 后删网格边界锯齿、主体误删、碎片化”的问题，移除了基于原始点云距离的几何后裁剪。

## 核心变化

- 删除 `trimUnsupportedPoissonRegions()` 及其支撑距离/保护环/连通块删除逻辑。
- UI 不再显示：保持原始扫描孔洞、孔洞支撑距离、自动支撑倍率、保护环、最小补洞区域面数。
- Poisson 等值面提取后仅使用官方输出 density 做保守筛选。
- `density_trim_percent` 默认从 1% 降为 0.5%。
- `scale` 默认从 1.10 改为 1.05，减少无数据区域外扩。
- 开启 `linear_fit`，改善提取顶点定位。
- “删除小连通网格”独立成开关，默认关闭。
- 基础 cleanup 只处理退化/重复/未引用数据，不主动裁复杂结构。

## 推荐参数

复杂家具/雕花扫描：

- Depth: 9（细节不足再 10）
- FullDepth: 5
- SamplesPerNode: 1.5
- PointWeight: 4.0
- Scale: 1.05
- Density Trim: 0.0 ~ 0.5%
- Linear Fit: 开
- 基础清理: 开
- 删除小连通网格: 关
- 保持输入点云尺寸: 开

如果想先判断纯 Poisson 质量，把 Density Trim 设为 0%，这样不会因为密度阈值删任何三角形。

注意：经典 Screened Poisson 本质上倾向闭合曲面。取消几何后裁剪后，不再承诺“绝对不补洞”；本版本优先保证主体表面稳定、连续、不被后处理破坏。
