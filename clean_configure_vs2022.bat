@echo off
setlocal

cd /d "%~dp0"

echo Cleaning CMake and Visual Studio generated directories...
if exist ".vs" rmdir /s /q ".vs"
if exist "build" rmdir /s /q "build"
if exist "out" rmdir /s /q "out"

echo Configuring JMEngine with Visual Studio 2022...
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
if errorlevel 1 (
    echo CMake configure failed.
    pause
    exit /b 1
)

echo Building RelWithDebInfo...
cmake --build build --config RelWithDebInfo --parallel
if errorlevel 1 (
    echo Build failed.
    pause
    exit /b 1
)

echo Build completed.
pause
