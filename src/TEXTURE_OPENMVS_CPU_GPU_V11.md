# OpenMVS-style CPU/GPU texture mapping V11

This version replaces the previous whole-camera-tile atlas with an OpenMVS-style patch pipeline while keeping the existing pceditor API and CPU/CUDA backends.

Pipeline:

1. CPU or CUDA rasterizes mesh visibility from calibrated camera views.
2. Each face gets multiple camera candidates scored by view angle, projected source-pixel area, image-center distance and border distance.
3. CPU/OpenMP performs edge-aware global Potts label optimization over mesh adjacency.
4. Connected faces carrying the same camera label become texture patches.
5. Each patch computes a source-image bounding rectangle in ORIGINAL RGB pixels.
6. Only those source rectangles are packed into the atlas. The original camera frame is no longer inserted as one giant tile.
7. Camera exposure gains are solved from colors sampled at real 3D patch seams.
8. Unobserved faces remain explicit neutral faces (camera id = -1); no arbitrary fallback camera is invented.

## CPU / CUDA division

- CUDA: depth rasterization, initial visibility and best-view pass. CUDA visibility depth is copied back and reused by CPU global candidate optimization.
- CPU/OpenMP: Top-K camera candidates, edge-aware global view selection, patch connected-components, atlas packing, color gain solve and final UV generation.
- CPU-only builds run the same final algorithm using CPU depth rasterization.

## Recommended Qt editor settings

The Qt editor now selects `Quality::OpenMVS` with:

- original RGB frames
- 1280 x 800 visibility buffer
- 8 view candidates per face
- 10 global label iterations
- 16384 maximum atlas
- 6 px patch border
- blurred frames are down-prioritized by selection logic instead of being hard removed in the scan workflow

## Important diagnostics

`Result` now exposes:

- `mappedTriangleCount`
- `unmappedTriangleCount`
- `texturePatchCount`
- `usedCameraCount`
- `triangleCameraIds`

If large front-facing regions remain unmapped after this version, verify the RGB frame/RT association and camera extrinsics before relaxing visibility tests.
