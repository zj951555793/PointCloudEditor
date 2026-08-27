# JMEngine V14

JMEngine is a cross-platform C++ point-cloud/mesh algorithm engine. V14 adds a **native Android Studio application with an OpenGL ES 3.1 renderer and GPU rectangle selection**, while keeping the reusable `jmengine` Android library module separate from the demo application.

## Repository layout

```text
JMEngine/
├─ include/JMEngine/                    public C++ API
├─ src/                                 core algorithms
├─ examples/qt_editor/                  Windows / RK3588 Qt demo
└─ android/native_app/                  native Android Studio project
   ├─ jmengine/                          reusable Android library (AAR + JNI + GLES)
   │  └─ src/main/
   │     ├─ java/com/jmengine/sdk/
   │     │  ├─ JMEngineNative.java      JNI algorithm facade
   │     │  └─ JMEngineGlesView.java    reusable GLES view
   │     └─ cpp/
   │        ├─ JMEngineAndroidJni.cpp
   │        └─ JMEngineGlesRenderer.cpp
   └─ app/                               native Android demo application
```

## Android architecture

```text
Native Android App (Java)
          ↓
      :jmengine AAR
          ↓
JMEngineGlesView + JMEngineNative
          ↓ JNI
libJMEngine_android.so
          ↓
JMEngine C++ core + GLES 3.1 renderer/selector
```

The Android UI does **not** depend on Qt. The same opaque native handle owns both `JMEngine::Engine` and the GLES renderer, so editing, rendering and selection always operate on the same soft-delete state.

## V14 Android features

The demo now supports:

- native GLES 3.1 point-cloud rendering;
- one-finger orbit;
- two-finger pinch zoom;
- Fit view;
- GPU Surface rectangle selection;
- GPU Through rectangle selection;
- selected-point highlight;
- delete selection;
- Undo / Redo;
- clear selection;
- export the current edited point-cloud state to PLY;
- PLY / OBJ document import through Android's document picker.

### Surface vs Through

`Surface` performs a GPU depth pre-pass and the compute shader only accepts points matching the current front depth. `Through` uses the same GLES compute shader but intentionally skips the depth test, so every projected point inside the rectangle can be selected.

Both modes therefore have independent semantics instead of sharing a CPU/GPU fallback flag.

## Open in Android Studio

Open:

```text
android/native_app
```

Android Studio will show:

```text
app        demo APK
jmengine   reusable Android library / AAR
```

Current configuration:

```text
ABI       arm64-v8a
minSdk    24
targetSdk 35
C++       C++17
Graphics  OpenGL ES 3.1+
JNI SO    libJMEngine_android.so
```

The app manifest declares OpenGL ES 3.1 as required. A modern Huawei ARM64 phone with GLES 3.1/3.2 can run this renderer directly.

## Build

Install Android SDK, NDK, CMake and JDK 17 in Android Studio, then Sync/Build the project. The Android project does not hard-code an NDK revision. If required, set one in `android/native_app/jmengine/build.gradle.kts`:

```kotlin
android {
    ndkVersion = "<installed side-by-side NDK revision>"
}
```

Typical outputs:

```text
android/native_app/app/build/outputs/apk/
android/native_app/jmengine/build/outputs/aar/
```

Production Android applications can use only the `jmengine` module:

```kotlin
dependencies {
    implementation(project(":jmengine"))
}
```

and create:

```java
JMEngineNative engine = new JMEngineNative();
JMEngineGlesView view = new JMEngineGlesView(context, engine);
```

## Android document access

Android document picker URIs are copied to the application's cache before they are passed to the portable C++ loader. Export writes the edited PLY into the application-specific external files directory, so broad storage permission is not required.

## Android native CMake

`android/native_app/jmengine/src/main/cpp/CMakeLists.txt` builds the portable `JMEngine` target and the Android JNI facade. Android automatically excludes desktop-only pieces such as DirectShow, Windows scanner SDK and CUDA. The Android shared library links `GLESv3`, `android` and `log`.

## Desktop build

Desktop usage remains:

```bash
cmake -S . -B build
cmake --build build --config Release
```

Windows keeps its existing OpenGL/CUDA/rulermvs path. RK3588 Linux can continue using the GLES path. Android adds a native Java + JNI front end without changing the public C++ algorithm API.

## Current V14 scope

V14 renders and edits point clouds. Android mesh rendering, Android texture mapping UI, Android base-plane interaction and in-process Android Poisson are intentionally not enabled yet. Those should be added on top of this stable native Android renderer rather than coupling the Android app back to Qt.

## V15 scan/render threading

The Qt scan pipeline keeps camera acquisition, SLAM and rendering delivery separated:

- Camera A/B capture runs on dedicated capture threads inside `ScanSourceWorker`.
- `ScanSourceWorker` and `RulerMvsWorker` run on separate `QThread`s.
- Raw frames are delivered Source -> SLAM directly without passing through the GUI queue.
- SLAM input is bounded/latest-frame-first to avoid unbounded latency when processing is slower than capture.
- GUI rendering uses a coalescing render mailbox: transient preview/current-frame/camera-pose data is latest-only, live pose updates are merged by frame id, and accepted persistent live frames are batched into a single GUI wake-up.
- Point history is uploaded once; subsequent live optimization changes only per-frame transforms rather than rebuilding the point VBO.

This design keeps the UI/OpenGL thread free of camera and SLAM computation and prevents queued rendering events from accumulating during SLAM bursts.

## 2026-08-21 scanner/observer camera consistency fix

The realtime scan observer now uses the exact same filtered visual pose as the scanner camera.
The pose is filtered once, converted to a single orthonormal quaternion basis, and then shared by
both the camera overlay and the OpenGL observer. The previous behind-camera offset and the second
paintGL follow EMA were removed, so observer eye/forward/up no longer lag the scanner visual pose.

## CUDA runtime deployment

On Windows, when the CUDA texture backend is enabled, CMake automatically deploys
the **minimal CUDA user-mode runtime** beside `JMEngine_qt_editor.exe`. The current
CUDA implementation directly links only `CUDA::cudart`, so the application `bin/`
contains only the matching `cudart64_*.dll` instead of a full CUDA Toolkit copy:

```
app/
  bin/
    JMEngine_qt_editor.exe
    cudart64_*.dll
```

The same DLL is installed by `cmake --install ...` into the install prefix `bin/`.
Set `-DJMENGINE_DEPLOY_CUDA_RUNTIME=OFF` to disable this deployment. `nvcuda.dll`
is never copied because it is supplied by the NVIDIA display driver, which remains
a prerequisite on the target machine. If future CUDA code directly links cuBLAS,
cuFFT, etc., add only those actually linked runtime DLLs to the deployment list.

### Camera model image orientation

`examples/qt_editor/config/camera_models.json` keeps separate A/B profiles. A
profile is valid only when its `model`, VID and PID all match the selected DirectShow
device. The optional `rotate` value is independent for each A/B profile and follows
OpenCV `cv::flip` semantics:

- `-1`: flip horizontally and vertically
- `0`: flip vertically
- `1`: flip horizontally
- `null` or omitted: no image transform

The transform is applied immediately after capture, before preview, camera
pairing, SLAM input, and raw scan recording. Camera-mode scanning refuses to
start when A/B has no matched model profile, when A/B selects the same device,
or when the two selected cameras resolve to different models. A/B `rotate` values may differ.

Virtual/dataset scanning never applies these camera-model `rotate` values. Dataset
`img/c` and `img/p` are passed to SLAM in their stored orientation. The Qt editor
shows an `img/c` preview during virtual scanning so dataset orientation can be
checked without modifying the virtual scan input.

### V20 virtual-scan reliability (JMC1S/JMC1L)

Virtual/dataset replay still **never applies camera-model rotate/flip**.  It now uses
lossless back-pressure instead of the physical-camera bounded drop policy, so a
slower JMC1S OneShot decode cannot silently remove consecutive frames.  `img/c`
and `img/p` are paired by their numeric frame id (or matching stem) before replay;
a missing file on one side is skipped rather than shifting every later pair.

Useful runtime diagnostics:

- `[VIRTUAL DATASET] ... paired=...` reports color/code counts and pairing.
- `[VIRTUAL DATASET] firstFrame color=... code=... rotate=disabled` confirms the
  exact unmodified input dimensions.
- `[SLAM CALIB] color=...` reports the RGB dimensions required by `calib.txt`.
- `[SLAM INPUT] RGB size mismatch ...` identifies a dataset/calibration mismatch.
- `[ONESHOT DECODE] ... points=...` shows whether the structured-light code frame
  actually decodes into geometry.

### V22 virtual replay stall fix

Virtual replay remains unrotated and lossless. The JMC1S path now limits the
lossless vendor-SDK frontier to two frames, serializes calls to the shared
`IOneShot::decode()` instance, and makes lossless waits interruptible during
stop/restart. Dataset filename pairing also falls back to sorted-index pairing
when a numeric/stem heuristic covers less than 80% of the recording, preventing
a weak match from silently truncating replay. Runtime diagnostics include
`[VIRTUAL BACKPRESSURE]`, replay progress, pairing mode, and source EOF.

## Windows self-contained runtime package

Windows Qt-editor builds now create a self-contained runtime package automatically.
After the normal `POST_BUILD` deployment finishes, CMake copies the complete editor
runtime directory and creates:

```text
build/package/windows/<CONFIG>/
  JMEngine_qt_editor/
  JMEngine_qt_editor-<CONFIG>.zip
```

The package captures the Windows runtime files already deployed by the build: Qt
plugins/DLLs and MSVC runtime from `windeployqt`, RulerMVS DLLs, bundled OpenCV
DLLs, the minimal CUDA `cudart64_*.dll` when CUDA deployment is enabled, the camera
model configuration, and the Poisson SurfaceTrimmer helper when that target exists.
It intentionally does not copy `nvcuda.dll`, because that DLL is supplied by the
installed NVIDIA display driver. Set `-DJMENGINE_PACKAGE_WINDOWS_RUNTIME=OFF` to
disable ZIP generation, or set `JMENGINE_WINDOWS_PACKAGE_ROOT` to change the output
directory.


### V23 atomic live-optimization rendering

Realtime SLAM pose refreshes no longer use the model-loading style incremental apply path.
One `getResults()` pose snapshot is treated as one optimization batch: all affected historical
frame points are transformed off-thread first, the complete batch is then committed to the CPU
history in one GUI pass, all pose dirty ranges are uploaded to the position/normal VBOs in the
same `paintGL()`, and the scene is drawn once. There is no `250000 points/paint` optimization
budget and no per-frame worker completion repaint, so an optimized history is never exposed
partially across several render frames. If a newer pose callback arrives while one batch is
running, its newest per-frame RTs are coalesced for the next complete batch.

### V24 fast atomic live-optimization rendering

V23's atomic display semantics are preserved: one SDK optimization snapshot still becomes visible
only after every affected historical pose has finished, and the scene is still drawn exactly once
for that completed batch. The old `livePosePointBudget` is **not** restored as a per-paint loading
budget.

The expensive work is now moved out of `paintGL()` instead:

- one optimization batch is transformed in parallel across the dedicated live-pose worker pool;
- workers write directly into final contiguous position/normal staging ranges;
- adjacent historical frame ranges are packed before the GUI refresh;
- `paintGL()` no longer allocates temporary position/normal arrays or rescans the full CPU cloud
  just to prepare VBO data;
- the GUI commits the completed CPU batch once, uploads the already-packed GPU ranges once, and
  draws once;
- runtime diagnostics `[LIVE POSE BATCH]` and `[LIVE POSE GPU]` report transform, GUI-commit, and
  GPU-upload costs when a batch is large or slow.

This keeps the required all-or-nothing optimization refresh while avoiding the v23 regression where
millions of optimized points were transformed serially by one worker and then repacked again on the
render thread.


### V25 OpenMP live-optimization transform

Realtime pose refresh keeps the V24 atomic rendering contract but replaces the multiple
QThreadPool transform workers with exactly one background batch task. When OpenMP is available,
that task parallelizes independent historical frames with `schedule(static)`. Live scanning uses
at most half of the logical CPUs (capped at 8 and also respecting the project CPU-1 policy), so
RGBDFusion/OneShot/capture/Qt keep CPU headroom. Small batches (fewer than 4 frames or fewer than 30000 transformed points)
remain serial to avoid OpenMP startup overhead. The complete batch still produces exactly one GUI
refresh, one CPU commit, one set of GPU range uploads and one scene draw; live optimization never
uses file-loading-style point budgets or partial renders. Runtime diagnostics print
`[LIVE POSE OMP] frames=... points=... ompThreads=... enabled=... transform=...ms`.
The Qt editor also no longer forces `/Od`/`-O0` in `RelWithDebInfo`; the standard optimized
RelWithDebInfo flags are retained together with debug symbols, which is important because the
render loop and live-pose staging code are compiled in `PointCloudWidget.cpp`.


### V26 automatic Desktop OpenGL 3.2 -> 2.1 fallback

The Windows/x86 Qt editor no longer hard-requires an OpenGL 3.2 context at process startup.
Before the main window is created it probes a 3.2 compatibility context on an offscreen surface.
If that succeeds, the existing modern path remains active (VAO + R32UI/geometry-shader GPU picking).
If it fails, the editor requests a Desktop OpenGL 2.1 context instead.

The legacy Desktop path only requires VBO + GLSL 1.20 for normal point/mesh rendering.
`GL_ARB_vertex_array_object` is now optional: when VAO is absent the backend rebinds the VBO
attribute layout for each draw instead of aborting initialization. Modern integer/geometry-shader
picking is disabled on the 2.1 path and selection automatically uses the existing CPU fallback.
Scanning, SLAM, live-pose OpenMP optimization, model processing, and export are unchanged.

For diagnostics, startup prints:

```text
[JMEngine GL] selected= OpenGL 3.2 compatibility ...
[JMEngine GL CAP] vao= true gpuPicking= true legacyVboFallback= false
```

or on an older adapter:

```text
[JMEngine GL] selected= OpenGL 2.1 legacy ...
[JMEngine GL CAP] vao= false gpuPicking= false legacyVboFallback= true
```

Set environment variable `JMENGINE_FORCE_GL21=1` to force the legacy path on a newer PC for
compatibility testing. The guaranteed Desktop minimum remains OpenGL 2.1; OpenGL 1.x / Microsoft
`GDI Generic` is treated as a missing/incorrect vendor GPU driver rather than a supported renderer.
