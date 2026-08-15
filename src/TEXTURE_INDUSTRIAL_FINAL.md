# Industrial Texture Mapping Final

This revision is the production-oriented texture pipeline for PointCloudEditor.

## Default pipeline

1. Validate camera/image/intrinsic data.
2. Reject severely blurred keyframes using relative Laplacian sharpness.
3. Build CPU/CUDA visibility and initial face-to-camera assignments.
4. Build Top-K camera candidates per triangle.
5. Optimize camera labels globally on mesh adjacency (Potts MRF / ICM).
6. Build camera-label boundaries and sample the same 3D boundary points in both images.
7. Solve robust per-camera RGB gains from those real seam correspondences.
8. Pack only used camera images into the atlas.
9. Add replicated atlas gutters so bilinear/mipmap sampling cannot bleed from neighbouring tiles.
10. Generate split texture mesh, UVs, preview colors and original-triangle camera diagnostics.

## Recommended production settings

```cpp
pceditor::texture::Config cfg;
cfg.backend = pceditor::texture::Backend::Auto;
cfg.quality = pceditor::texture::Quality::High;
cfg.maxKeyframes = 200;
cfg.maxAtlasSize = 8192;
cfg.visibilityWidth = 320;
cfg.visibilityHeight = 240;
cfg.globalViewSelection = true;
cfg.candidateCameraCount = 4;
cfg.globalViewIterations = 5;
cfg.globalSmoothness = 0.18f;
cfg.rejectBlurredFrames = true;
cfg.minRelativeSharpness = 0.18f;
cfg.exposureCompensation = true;
cfg.seamAwareExposureCompensation = true;
cfg.exposureSolveIterations = 24;
cfg.atlasPaddingPixels = 8;
```

Use `Quality::Ultra` for offline final export. It performs more global view-label iterations and stronger continuity regularization.

## Result diagnostics

`Result` now exposes:

- `inputCameraCount`
- `acceptedCameraCount`
- `usedCameraCount`
- `mappedTriangleCount`
- `triangleCameraIds`

These values should be logged in the UI/scan pipeline for production diagnostics.

## Tests

The texture unit test additionally verifies atlas gutter safety, UV containment, severe blur rejection and result counters. The tiny dataset integration test remains enabled.

