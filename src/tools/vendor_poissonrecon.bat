@echo off
setlocal
where py >nul 2>nul
if %ERRORLEVEL%==0 (
    py -3 "%~dp0vendor_poissonrecon.py" %*
) else (
    python "%~dp0vendor_poissonrecon.py" %*
)
exit /b %ERRORLEVEL%
