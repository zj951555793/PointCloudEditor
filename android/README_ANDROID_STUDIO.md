# PointCloudEditor V12 - Android Studio / Qt for Android

This Android port keeps the same Qt Widgets editor and uses the existing OpenGL ES 3.1 renderer.
The target ABI is **arm64-v8a**.

## Required local tools

- Android Studio (install Android SDK + NDK + CMake/Ninja)
- Qt 6 Android arm64-v8a kit containing `bin/qt-cmake.bat`
- JDK required by your Android Studio / Qt installation

## Configure from Windows

From the repository root:

```bat
android\configure_android_studio.bat ^
  C:\Qt\6.8.3\android_arm64_v8a ^
  C:\Users\YOUR_NAME\AppData\Local\Android\Sdk ^
  C:\Users\YOUR_NAME\AppData\Local\Android\Sdk\ndk\YOUR_NDK_VERSION
```

Then build the APK/Gradle project:

```bat
android\build_apk.bat
```

Qt creates the Android Gradle project in `build-android-arm64/android-build` during APK packaging.
Open that generated `android-build` directory in Android Studio for install/logcat/debug packaging work.
Keep C++/CMake edits in the repository source tree; `android-build` is generated output.

## V12 Android feature state

Available in this version:

- Qt Widgets main editor
- OpenGL ES 3.1 renderer
- touch rotation / pan / zoom / editing
- point-cloud and mesh loading/editing/export
- CPU surface/through selection fallback
- base-plane fit/cut tool
- CPU OpenMVS-style texture mapping path

Platform-specific behavior:

- CUDA is disabled on Android.
- Windows DirectShow/COM libraries are never linked on Android.
- Windows `rulermvs`/OpenCV bundle is never linked on Android.
- Camera acquisition on Android is not yet connected to Camera2/Qt Multimedia; the editor still builds and model editing works.
- Industrial Poisson is disabled for V12 Android because the current integration launches the official `SurfaceTrimmer` helper executable. Android needs the trimmer moved in-process before enabling this safely.
- GLES GPU picking/compute is the next step; this V12 baseline uses GLES for rendering and falls back to the CPU selector where the current GLES backend does not expose GPU picking.

## GLES requirement

The editor requests an OpenGL ES **3.1** context because the shared RK3588/Android shader path uses GLSL ES 3.10.
Use a device whose GPU/driver exposes GLES 3.1 or newer.

## Android Studio notes

Do not replace the Qt-generated Gradle files with a plain native Android template. Qt's `androiddeployqt` injects Qt libraries, plugins, Java bootstrap code and native deployment metadata. Android Studio is best used to open the generated `android-build` project after CMake/Qt configuration.
