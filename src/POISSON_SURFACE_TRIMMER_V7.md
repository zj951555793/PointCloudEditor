# Poisson SurfaceTrimmer V7

## 本版变化

- 删除工业泊松面板中的“法向一致化”和“一致化半径”。
- Poisson 阶段不再翻转已有法线；输入法线方向原样传入。缺法线时仍可使用 KNN/PCA 补算法线。
- 删除 V6 的简化 density face reject。
- 使用 PoissonRecon SurfaceTrimmer 的核心 SplitPolygon 语义：
  - `density > Trim` 的区域保留；
  - 三角形跨越阈值时，在边上按 density 线性插值生成切割顶点；
  - 同一条原始边复用切割顶点，保证切口连续；
  - 裁剪后的多边形重新三角化。
- `SurfaceTrimmer Trim` 默认 5.0。Trim 越大，裁剪越强；0 表示不裁剪。
- “删除小岛”默认关闭，避免误删雕花/薄结构；需要时可显式开启。

## 推荐起始参数

对于 Depth 约 9~10 的家具扫描：

- SurfaceTrimmer Trim: 5.0
- 若底部仍保留：6.0 / 7.0
- 若真实边缘被裁掉：4.0 / 4.5
- SurfaceTrimmer 删除小岛：关闭
- 岛面积比：0.001（仅在显式删除小岛时生效）

注意：官方 PoissonRecon 的 density 输出代表等值面顶点的估计采样深度，所以 `Trim=1` 是很宽松的阈值，并不是“更强裁剪”。
