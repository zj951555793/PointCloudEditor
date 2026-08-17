@echo off
setlocal
set "BUILD=%~1"
if "%BUILD%"=="" set "BUILD=build"
ctest --test-dir "%BUILD%" -R JMEngine_texture_bop --output-on-failure
exit /b %ERRORLEVEL%
