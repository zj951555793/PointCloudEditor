# Poisson live-scan snapshot fix V11.1

## Root cause
`runProcessingOperation()` previously captured the scene-owned `shared_ptr<PointCloud>` and passed that same object to the background Poisson worker. `editBusy_` prevents editor operations, but it does not make the live scan producer / real-time pose optimizer stop owning or updating that object. Poisson could therefore read a cloud while it was being updated.

## Fix
- Scene pointers are retained only as identity tokens for stale-result validation.
- A full PointCloud/TriangleMesh deep copy is made before the worker starts.
- The worker receives only this immutable snapshot.
- Snapshot construction does not compact/remap/cleanup/recalculate normals/apply transforms.
- The generated Poisson mesh is still a new model and never modifies the source scan.

This change is independent of the OpenMVS-style texture mapper.
