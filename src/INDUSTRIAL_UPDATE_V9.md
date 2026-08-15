# Industrial Update V9

## 1. Poisson SurfaceTrimmer
- SurfaceTrimmer default Trim changed to 7.0.
- Existing official PoissonRecon/SurfaceTrimmer integration retained.

## 2. Camera persistence
- Selecting an existing model no longer calls fitView().
- Creating texture/Poisson/processing result models no longer calls fitView().
- Offline reconstruction replacement no longer calls fitView().
- Only the first loaded model is auto-fitted; F remains the explicit Fit View command.

## 3. Texture mapping
- Scan texture keyframes are captured from the original undistorted RGB resolution (scale divisor forced to 1).
- Texture mapping UI path uses Ultra quality, 960x600 visibility buffers, 8 candidate cameras, and 16384 maximum atlas dimension.
- Atlas packing preserves source image aspect ratio and keeps original image resolution whenever it fits.
- Occlusion rejects only surfaces clearly behind the visibility depth, with a depth-relative tolerance.
- Hard blur-frame rejection is disabled in the scan texturing path so a unique recessed view is never discarded solely for sharpness.
- Isolated unmapped faces can recover a camera from adjacent texture patches after geometric validation.
- Genuinely untextured triangles are retained with a neutral fallback atlas region instead of being removed from the generated mesh (no visual triangle holes).

## 4. Export
- File export and Ctrl+S quick save run in the existing worker thread pool.
- Editing is frozen while an export snapshot is being written, preventing concurrent topology/data changes.
- Texture OBJ export respects the current triangle-active mask.
- If textured topology no longer matches the stored UV topology, export fails explicitly and asks for texture remapping instead of writing a corrupt OBJ.
- Export action is disabled while the background export is active.

## Validation
- pceditor_core_tests: passed
- pceditor_texture_tests: passed
- The current Linux container does not contain Qt6 Widgets/OpenGL, so the Qt editor target cannot be compiled here; core/texture code builds and tests pass.
