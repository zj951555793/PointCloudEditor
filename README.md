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
