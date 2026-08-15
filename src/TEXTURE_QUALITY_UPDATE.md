# Texture quality update

This revision upgrades texture camera assignment from per-triangle greedy selection to a global, OpenMVS-style quality pipeline.

## New quality modes

```cpp
pceditor::texture::Config cfg;
cfg.quality = pceditor::texture::Quality::High; // default
```

- `Fast`: greedy camera selection + isolated-label smoothing.
- `High`: visibility-aware top-K candidates + global mesh label optimization.
- `Ultra`: more global optimization passes and stronger patch continuity.

## Main changes

- full triangle-in-image validation
- perspective-correct CPU visibility depth
- projected pixel-area score
- image-border quality score
- global face/camera candidate generation
- mesh-adjacency Potts/ICM label optimization
- continuous camera patches instead of A/B/A/C face speckle
- existing exposure compensation retained
- CPU/CUDA initial selector retained; High/Ultra revalidate candidate visibility

## Recommended scan configuration

```cpp
Config cfg;
cfg.backend = Backend::Auto;
cfg.quality = Quality::High;
cfg.globalViewSelection = true;
cfg.candidateCameraCount = 4;
cfg.globalViewIterations = 5;
cfg.globalSmoothness = 0.18f;
cfg.visibilityWidth = 320;
cfg.visibilityHeight = 240;
```

For final export, try `Quality::Ultra`. For interactive preview use `Quality::High` or `Fast`.
