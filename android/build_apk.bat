@echo off
setlocal
set "BUILD_DIR=%~dp0..\build-android-arm64"
if not exist "%BUILD_DIR%\CMakeCache.txt" (
  echo [ERROR] Configure Android first with android\configure_android_studio.bat
  exit /b 2
)
cmake --build "%BUILD_DIR%" --target apk
if errorlevel 1 exit /b %errorlevel%
echo.
echo APK build finished. Qt's generated Gradle project is normally under:
echo   %BUILD_DIR%\android-build
