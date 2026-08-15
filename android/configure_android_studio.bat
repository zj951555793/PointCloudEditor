@echo off
setlocal EnableExtensions

if "%~1"=="" goto :usage
if "%~2"=="" goto :usage
if "%~3"=="" goto :usage

set "QT_ANDROID=%~1"
set "ANDROID_SDK_ROOT=%~2"
set "ANDROID_NDK_ROOT=%~3"
set "SRC_DIR=%~dp0.."
set "BUILD_DIR=%SRC_DIR%\build-android-arm64"

if not exist "%QT_ANDROID%\bin\qt-cmake.bat" (
  echo [ERROR] qt-cmake.bat not found: %QT_ANDROID%\bin\qt-cmake.bat
  exit /b 2
)

set "ANDROID_HOME=%ANDROID_SDK_ROOT%"
set "ANDROID_NDK_HOME=%ANDROID_NDK_ROOT%"

call "%QT_ANDROID%\bin\qt-cmake.bat" -S "%SRC_DIR%" -B "%BUILD_DIR%" -GNinja ^
  -DANDROID_ABI=arm64-v8a ^
  -DANDROID_PLATFORM=android-26 ^
  -DPCEDITOR_BUILD_QT_EDITOR=ON ^
  -DPCEDITOR_RENDER_BACKEND=GLES31 ^
  -DPCEDITOR_ENABLE_TEXTURE_MAPPING=ON ^
  -DPCEDITOR_ENABLE_TEXTURE_CUDA=OFF ^
  -DPCEDITOR_ENABLE_POISSONRECON=OFF ^
  -DPCEDITOR_BUILD_TESTS=OFF ^
  -DPCEDITOR_BUILD_EXAMPLES=OFF
if errorlevel 1 exit /b %errorlevel%

cmake --build "%BUILD_DIR%" --target pceditor_qt_editor -j 8
if errorlevel 1 exit /b %errorlevel%

echo.
echo [OK] Native Android target configured and built.
echo Build dir: %BUILD_DIR%
echo.
echo Next, create the Qt Gradle project/APK with:
echo   cmake --build "%BUILD_DIR%" --target apk

echo After that, open this folder in Android Studio if it exists:
echo   %BUILD_DIR%\android-build
exit /b 0

:usage
echo Usage:
echo   configure_android_studio.bat ^<Qt-Android-arm64-dir^> ^<Android-SDK-dir^> ^<Android-NDK-dir^>
echo Example:
echo   configure_android_studio.bat C:\Qt\6.8.3\android_arm64_v8a C:\Users\me\AppData\Local\Android\Sdk C:\Users\me\AppData\Local\Android\Sdk\ndk\27.2.12479018
exit /b 1
