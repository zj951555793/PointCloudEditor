@echo off
setlocal EnableExtensions EnableDelayedExpansion

if "%~1"=="" goto :usage
if "%~2"=="" goto :usage
if "%~3"=="" goto :usage

set "QT_ANDROID=%~1"
set "ANDROID_SDK_ROOT=%~2"
set "ANDROID_NDK_ROOT=%~3"
set "SRC_DIR=%~dp0.."
set "BUILD_DIR=%SRC_DIR%\build-android-arm64"
set "NINJA_EXE="

if not exist "%QT_ANDROID%\bin\qt-cmake.bat" (
  echo [ERROR] qt-cmake.bat not found:
  echo         %QT_ANDROID%\bin\qt-cmake.bat
  exit /b 2
)

if not exist "%ANDROID_SDK_ROOT%" (
  echo [ERROR] Android SDK not found:
  echo         %ANDROID_SDK_ROOT%
  exit /b 2
)

if not exist "%ANDROID_NDK_ROOT%" (
  echo [ERROR] Android NDK not found:
  echo         %ANDROID_NDK_ROOT%
  exit /b 2
)

set "ANDROID_HOME=%ANDROID_SDK_ROOT%"
set "ANDROID_NDK_HOME=%ANDROID_NDK_ROOT%"

echo [INFO] Searching for ninja.exe ...

rem ---------------------------------------------------------------------------
rem 1. Ninja already available in PATH
rem ---------------------------------------------------------------------------
for /f "delims=" %%I in ('where ninja.exe 2^>nul') do (
  if not defined NINJA_EXE set "NINJA_EXE=%%~fI"
)
if defined NINJA_EXE goto :ninja_found

rem ---------------------------------------------------------------------------
rem 2. Ninja installed by Qt Maintenance Tool.
rem    Example: C:\Qt\Tools\Ninja\ninja.exe
rem    QT_ANDROID normally looks like C:\Qt\6.8.3\android_arm64_v8a.
rem ---------------------------------------------------------------------------
for %%I in ("%QT_ANDROID%\..\..") do set "QT_ROOT=%%~fI"
if exist "!QT_ROOT!\Tools\Ninja\ninja.exe" (
  set "NINJA_EXE=!QT_ROOT!\Tools\Ninja\ninja.exe"
  goto :ninja_found
)

rem Some Qt installations keep Ninja next to Qt Creator tools.
if exist "!QT_ROOT!\Tools\QtCreator\bin\ninja.exe" (
  set "NINJA_EXE=!QT_ROOT!\Tools\QtCreator\bin\ninja.exe"
  goto :ninja_found
)

rem ---------------------------------------------------------------------------
rem 3. Ninja bundled with an Android SDK CMake package.
rem    Android Studio SDK Manager -> SDK Tools -> CMake installs it here.
rem ---------------------------------------------------------------------------
if exist "%ANDROID_SDK_ROOT%\cmake" (
  for /f "delims=" %%I in ('where /r "%ANDROID_SDK_ROOT%\cmake" ninja.exe 2^>nul') do (
    if not defined NINJA_EXE set "NINJA_EXE=%%~fI"
  )
)
if defined NINJA_EXE goto :ninja_found

rem ---------------------------------------------------------------------------
rem 4. Fallback to the standard per-user Android SDK location.
rem ---------------------------------------------------------------------------
if exist "%LOCALAPPDATA%\Android\Sdk\cmake" (
  for /f "delims=" %%I in ('where /r "%LOCALAPPDATA%\Android\Sdk\cmake" ninja.exe 2^>nul') do (
    if not defined NINJA_EXE set "NINJA_EXE=%%~fI"
  )
)
if defined NINJA_EXE goto :ninja_found

rem ---------------------------------------------------------------------------
rem 5. Last-resort common Android Studio installation search.
rem ---------------------------------------------------------------------------
if exist "%ProgramFiles%\Android\Android Studio" (
  for /f "delims=" %%I in ('where /r "%ProgramFiles%\Android\Android Studio" ninja.exe 2^>nul') do (
    if not defined NINJA_EXE set "NINJA_EXE=%%~fI"
  )
)
if defined NINJA_EXE goto :ninja_found

echo.
echo [ERROR] ninja.exe was not found automatically.
echo.
echo Install one of the following and run this script again:
echo   1. Qt Maintenance Tool ^> Developer and Designer Tools ^> Ninja
echo   2. Android Studio ^> SDK Manager ^> SDK Tools ^> CMake
echo.
echo Locations checked:
echo   PATH
echo   !QT_ROOT!\Tools\Ninja\ninja.exe
echo   %ANDROID_SDK_ROOT%\cmake\...\bin\ninja.exe
echo   %LOCALAPPDATA%\Android\Sdk\cmake\...\bin\ninja.exe
echo.
exit /b 3

:ninja_found
echo [OK] Ninja found:
echo      !NINJA_EXE!
"!NINJA_EXE!" --version
if errorlevel 1 (
  echo [ERROR] ninja.exe was found but cannot be executed.
  exit /b 4
)

echo.
echo [INFO] Configuring Android arm64 project ...

call "%QT_ANDROID%\bin\qt-cmake.bat" -S "%SRC_DIR%" -B "%BUILD_DIR%" -G Ninja ^
  "-DCMAKE_MAKE_PROGRAM=!NINJA_EXE!" ^
  -DANDROID_ABI=arm64-v8a ^
  -DANDROID_PLATFORM=android-26 ^
  -DJMENGINE_BUILD_QT_EDITOR=ON ^
  -DJMENGINE_RENDER_BACKEND=GLES31 ^
  -DJMENGINE_ENABLE_TEXTURE_MAPPING=ON ^
  -DJMENGINE_ENABLE_TEXTURE_CUDA=OFF ^
  -DJMENGINE_ENABLE_POISSONRECON=OFF ^
  -DJMENGINE_BUILD_TESTS=OFF ^
  -DJMENGINE_BUILD_EXAMPLES=OFF
if errorlevel 1 exit /b %errorlevel%

rem Use the exact Ninja that configured CMake instead of relying on PATH.
"!NINJA_EXE!" -C "%BUILD_DIR%" JMEngine_qt_editor
if errorlevel 1 exit /b %errorlevel%

echo.
echo [OK] Native Android target configured and built.
echo Build dir: %BUILD_DIR%
echo Ninja:     !NINJA_EXE!
echo.
echo Next, generate the Qt Gradle project/APK with:
echo   android\build_apk.bat
echo.
echo After APK generation, open this folder in Android Studio:
echo   %BUILD_DIR%\android-build
exit /b 0

:usage
echo Usage:
echo   configure_android_studio.bat ^<Qt-Android-arm64-dir^> ^<Android-SDK-dir^> ^<Android-NDK-dir^>
echo.
echo Example:
echo   configure_android_studio.bat C:\Qt\6.8.3\android_arm64_v8a C:\Users\me\AppData\Local\Android\Sdk C:\Users\me\AppData\Local\Android\Sdk\ndk\27.2.12479018
echo.
echo Ninja does not need to be added to PATH. This script searches for it automatically.
exit /b 1
