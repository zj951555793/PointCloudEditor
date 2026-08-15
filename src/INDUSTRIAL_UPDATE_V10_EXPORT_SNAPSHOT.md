# Industrial V10 - Export snapshot isolation

This patch intentionally changes only the export path.

## Problem fixed
V9 background export captured the scene `shared_ptr<PointCloud>`, `shared_ptr<TriangleMesh>` and texture result directly. That is not a snapshot: the export worker and the scene still referred to the same objects. Any future export-side preprocessing or concurrent model replacement could therefore make scene/export state diverge or race.

## New rule
Export never receives a scene-owned mutable object.

The worker first creates a deep snapshot containing:
- every `Point` (position/color/normal/flags),
- mesh indices in the original order,
- triangle flags in the original order,
- texture split vertices,
- texture UVs, atlas, indices and camera labels.

Snapshot creation does **not** call compact, cleanup, vertex remap, triangle remap, normal rebuild, or UV rebuild.

The exporter may only read/modify its local snapshot. Scene Poisson mesh data is never passed into `ModelIO::save()` or texture `saveObj()`.

## Concurrency
Export is rejected while processing/diagnostics/editing is active. Editing remains locked until the background export completes, ensuring the snapshot corresponds to one well-defined scene state.
